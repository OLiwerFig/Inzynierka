#include "multiplanedetector.h"
#include "draw.h"
#include <QDebug>
#include <set>
#include <algorithm>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

bool MultiPlaneDetector::isValidSensorData(const QList<QList<int>>& sensorData) {
    if (sensorData.size() != 8) return false;
    for (const auto& row : sensorData) {
        if (row.size() != 8) return false;
        for (int value : row) {
            if (value <= 0 || value > 4000) return false;
        }
    }
    return true;
}

std::vector<MultiPlaneDetector::Plane> MultiPlaneDetector::detectMultiplePlanes(const QList<QList<int>> &sensorData) {
    std::vector<Plane> detectedPlanes;

    if (!isValidSensorData(sensorData)) {
        qDebug() << "Nieprawidłowe dane czujnika";
        return detectedPlanes;
    }

    // Obrót danych o 90 stopni w prawo i uznanie ich za "oryginalne"
    QList<QList<int>> rotatedData = Draw::rotate90Right(sensorData);

    // Konwersja obróconych danych do 3D
    std::vector<Draw::PointInfo> points;
    try {
        points = Draw::calculateCoordinatesWithIndices(rotatedData);
    } catch (const std::exception &e) {
        qDebug() << "Błąd przy obliczaniu współrzędnych 3D:" << e.what();
        return detectedPlanes;
    }

    if (points.size() < 3) {
        qDebug() << "Za mało punktów do wykrycia płaszczyzny";
        return detectedPlanes;
    }

    const int maxIterations = 300;
    double distanceThreshold = 5.0;
    const int minInliers = 15;
    const int maxPlanes = 4;

    std::vector<Draw::PointInfo> currentCloud = points;

    for (int p = 0; p < maxPlanes; p++) {
        Plane bestPlane;
        int bestInlierCount = 0;
        std::vector<int> bestInliers;

        for (int iter = 0; iter < maxIterations; iter++) {
            if (currentCloud.size() < 3) break;
            std::set<int> chosenIndices;
            while (chosenIndices.size() < 3) {
                chosenIndices.insert(rand() % currentCloud.size());
            }

            std::vector<int> indices(chosenIndices.begin(), chosenIndices.end());
            Eigen::Vector3d p1(currentCloud[indices[0]].x, currentCloud[indices[0]].y, currentCloud[indices[0]].z);
            Eigen::Vector3d p2(currentCloud[indices[1]].x, currentCloud[indices[1]].y, currentCloud[indices[1]].z);
            Eigen::Vector3d p3(currentCloud[indices[2]].x, currentCloud[indices[2]].y, currentCloud[indices[2]].z);

            Eigen::Vector3d v1 = p2 - p1;
            Eigen::Vector3d v2 = p3 - p1;
            Eigen::Vector3d normal = v1.cross(v2);
            double norm = normal.norm();
            if (norm < 1e-9) continue;
            normal.normalize();
            double d = -normal.dot(p1);

            std::vector<double> distances(currentCloud.size());
            for (size_t i = 0; i < currentCloud.size(); i++) {
                double dist = std::abs(normal(0)*currentCloud[i].x + normal(1)*currentCloud[i].y + normal(2)*currentCloud[i].z + d);
                distances[i] = dist;
            }

            std::vector<int> inliers;
            for (size_t i=0; i<distances.size(); i++) {
                if (distances[i] < distanceThreshold) {
                    inliers.push_back((int)i);
                }
            }

            if ((int)inliers.size() > bestInlierCount) {
                bestInlierCount = (int)inliers.size();
                bestInliers = inliers;
                bestPlane.params.a = normal(0);
                bestPlane.params.b = normal(1);
                bestPlane.params.c = normal(2);
                bestPlane.params.d = d;
            }
        }

        if (bestInlierCount < minInliers) {
            break;
        }

        // Dostosuj próg wg mediany
        {
            std::vector<double> inlierDistances;
            inlierDistances.reserve(bestInlierCount);
            for (int idx : bestInliers) {
                double dist = std::abs(bestPlane.params.a*currentCloud[idx].x
                                       +bestPlane.params.b*currentCloud[idx].y
                                       +bestPlane.params.c*currentCloud[idx].z
                                       +bestPlane.params.d);
                inlierDistances.push_back(dist);
            }
            std::sort(inlierDistances.begin(), inlierDistances.end());
            double medianDist = inlierDistances[inlierDistances.size()/2];
            double adaptiveFactor = 2.0;
            distanceThreshold = std::max(distanceThreshold, medianDist * adaptiveFactor);

            // Refine inliers
            std::vector<int> refinedInliers;
            for (int idx : bestInliers) {
                double dist = std::abs(bestPlane.params.a*currentCloud[idx].x
                                       +bestPlane.params.b*currentCloud[idx].y
                                       +bestPlane.params.c*currentCloud[idx].z
                                       +bestPlane.params.d);
                if (dist < distanceThreshold) refinedInliers.push_back(idx);
            }
            bestInliers = refinedInliers;
            bestInlierCount = (int)bestInliers.size();
        }

        double sumDist=0;
        for (int idx : bestInliers) {
            double dist=std::abs(bestPlane.params.a*currentCloud[idx].x
                                   +bestPlane.params.b*currentCloud[idx].y
                                   +bestPlane.params.c*currentCloud[idx].z
                                   +bestPlane.params.d);
            sumDist+=dist;
        }
        bestPlane.meanResidual = sumDist / bestInlierCount;

        double azimuth = std::atan2(bestPlane.params.a, bestPlane.params.c)*180.0/M_PI;
        bestPlane.params.azimuth_angle=azimuth;
        bestPlane.isLeftPlane=(azimuth<0);

        // Zapisz wiersze i kolumny dla tej płaszczyzny (już w obróconym układzie)
        {
            std::set<int> rowSet, colSet;
            for (int idx : bestInliers) {
                int r = currentCloud[idx].row;
                int c = currentCloud[idx].col;
                rowSet.insert(r);
                colSet.insert(c);
            }

            bestPlane.rowsUsed.assign(rowSet.begin(), rowSet.end());
            bestPlane.colsUsed.assign(colSet.begin(), colSet.end());
        }

        detectedPlanes.push_back(bestPlane);

        // Usuń inliers
        std::set<int> inlierSet(bestInliers.begin(), bestInliers.end());
        std::vector<Draw::PointInfo> newCloud;
        newCloud.reserve(currentCloud.size()-bestInliers.size());
        for (int i=0; i<(int)currentCloud.size(); i++) {
            if (inlierSet.find(i)==inlierSet.end()) {
                newCloud.push_back(currentCloud[i]);
            }
        }
        currentCloud = newCloud;
        if ((int)currentCloud.size()<minInliers) break;
    }

    if (detectedPlanes.empty()) {
        qDebug()<<"Brak wykrytych płaszczyzn";
    }

    return detectedPlanes;
}
