#include "multiplanedetector.h"
#include "draw.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <cmath>
#include <QDebug>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

std::vector<Point3D> MultiPlaneDetector::calculateCoordinates(
    const QList<QList<int>>& sensorData,
    const double elevationAngles[8][8],
    const double azimuthAngles[8][8]) {

    std::vector<Point3D> coords;
    coords.reserve(64);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            double d_value = sensorData[i][j];
            double elev = elevationAngles[i][j] * M_PI / 180.0;
            double azim = azimuthAngles[i][j] * M_PI / 180.0;

            Point3D p;
            p.x = d_value * std::cos(elev) * std::sin(azim);
            p.y = d_value * std::sin(elev);
            p.z = d_value * std::cos(elev) * std::cos(azim);
            p.row = i;
            p.col = j;

            coords.push_back(p);
        }
    }
    return coords;
}

void MultiPlaneDetector::fitPlane(
    const std::vector<Point3D>& coords,
    double& a, double& b, double& c, double& d) {

    int N = coords.size();

    // Compute sums
    double S_xx = 0, S_xy = 0, S_xz = 0, S_yy = 0, S_yz = 0, S_zz = 0;
    double S_x1 = 0, S_y1 = 0, S_z1 = 0;

    for (const auto& p : coords) {
        S_xx += p.x * p.x;
        S_xy += p.x * p.y;
        S_xz += p.x * p.z;
        S_yy += p.y * p.y;
        S_yz += p.y * p.z;
        S_zz += p.z * p.z;
        S_x1 += p.x;
        S_y1 += p.y;
        S_z1 += p.z;
    }

    // Build matrices
    Eigen::Matrix3d M;
    M << S_xx, S_xy, S_xz,
        S_xy, S_yy, S_yz,
        S_xz, S_yz, S_zz;

    Eigen::Vector3d k(S_x1, S_y1, S_z1);

    // Modified matrix M'
    Eigen::Matrix3d M_prime = M - (k * k.transpose()) / N;

    // Solve eigenvalue problem
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M_prime);

    // Find smallest eigenvalue
    int idx = 0;
    double min_eigenval = solver.eigenvalues()[0];
    for (int i = 1; i < 3; i++) {
        if (solver.eigenvalues()[i] < min_eigenval) {
            min_eigenval = solver.eigenvalues()[i];
            idx = i;
        }
    }

    // Get normal vector
    Eigen::Vector3d normal = solver.eigenvectors().col(idx);

    // Compute d
    d = -(normal[0] * S_x1 + normal[1] * S_y1 + normal[2] * S_z1) / N;

    // Normalize coefficients
    double norm = normal.norm();
    normal /= norm;
    d /= norm;

    a = normal[0];
    b = normal[1];
    c = normal[2];
}

std::vector<size_t> MultiPlaneDetector::identifyOutliers(
    const std::vector<Point3D>& coords,
    double a, double b, double c, double d,
    double threshold_factor) {

    std::vector<double> residuals;
    residuals.reserve(coords.size());

    // Calculate residuals
    for (const auto& p : coords) {
        double dist = std::abs(a * p.x + b * p.y + c * p.z + d);
        residuals.push_back(dist);
    }

    // Calculate mean and standard deviation
    double mean_residual = 0;
    for (double r : residuals) mean_residual += r;
    mean_residual /= residuals.size();

    double variance = 0;
    for (double r : residuals) {
        double diff = r - mean_residual;
        variance += diff * diff;
    }
    variance /= residuals.size();
    double std_residual = std::sqrt(variance);

    // Identify outliers
    double threshold = mean_residual + threshold_factor * std_residual;
    std::vector<size_t> outlier_indices;

    for (size_t i = 0; i < residuals.size(); i++) {
        if (residuals[i] > threshold) {
            outlier_indices.push_back(i);
        }
    }

    return outlier_indices;
}

bool MultiPlaneDetector::isIncreasingTrend(const std::vector<Point3D>& coords) {
    // Grupujemy punkty według kolumn
    std::vector<std::vector<double>> columns(8);
    for (const auto& p : coords) {
        columns[p.col].push_back(p.z);
    }

    // Obliczamy średni trend dla każdej kolumny
    double total_diff = 0;
    int count = 0;

    for (size_t i = 0; i < columns.size() - 1; i++) {
        if (!columns[i].empty() && !columns[i + 1].empty()) {
            double avg_curr = std::accumulate(columns[i].begin(), columns[i].end(), 0.0) / columns[i].size();
            double avg_next = std::accumulate(columns[i + 1].begin(), columns[i + 1].end(), 0.0) / columns[i + 1].size();
            total_diff += (avg_next - avg_curr);
            count++;
        }
    }

    return count > 0 && (total_diff / count) > 0;
}

std::vector<MultiPlaneDetector::DataSegment> MultiPlaneDetector::analyzeTrends(const std::vector<Point3D>& coords) {
    std::vector<DataSegment> segments;
    if (coords.empty()) return segments;

    // Grupowanie punktów według wierszy
    std::vector<std::vector<Point3D>> rowGroups(8);
    for (const auto& p : coords) {
        rowGroups[p.row].push_back(p);
    }

    // Obliczanie średnich Z dla każdego wiersza
    std::vector<double> rowAverages(8, 0.0);
    double maxAverage = 0.0;
    for (int i = 0; i < 8; i++) {
        if (!rowGroups[i].empty()) {
            double sum = 0.0;
            for (const auto& p : rowGroups[i]) {
                sum += p.z;
            }
            rowAverages[i] = sum / rowGroups[i].size();
            maxAverage = std::max(maxAverage, std::abs(rowAverages[i]));
        }
    }

    // Procent zmiany potrzebny do uznania za zmianę trendu (np. 15%)
    const double TREND_CHANGE_THRESHOLD = 0.05;

    int currentStartRow = 0;
    bool currentTrend = true;

    // Wykrywanie zmian trendu
    for (int i = 1; i < 8; i++) {
        if (rowGroups[i].empty() || rowGroups[i-1].empty()) continue;

        double diff = rowAverages[i] - rowAverages[i-1];
        double percentChange = std::abs(diff) / maxAverage; // Zmiana procentowa
        bool newTrend = (diff > 0);

        if (i == 1) {
            currentTrend = newTrend;
        } else if (currentTrend != newTrend && percentChange > TREND_CHANGE_THRESHOLD) {
            // Tworzenie nowego segmentu
            DataSegment segment;
            segment.startRow = currentStartRow;
            segment.endRow = i-1;
            segment.isIncreasing = currentTrend;

            // Dodawanie punktów do segmentu
            for (int row = segment.startRow; row <= segment.endRow; row++) {
                segment.points.insert(
                    segment.points.end(),
                    rowGroups[row].begin(),
                    rowGroups[row].end()
                    );
            }

            if (!segment.points.empty()) {
                segments.push_back(segment);
            }

            currentStartRow = i;
            currentTrend = newTrend;
        }
    }

    // Dodanie ostatniego segmentu
    if (currentStartRow < 7) {
        DataSegment segment;
        segment.startRow = currentStartRow;
        segment.endRow = 7;
        segment.isIncreasing = currentTrend;

        for (int row = segment.startRow; row <= segment.endRow; row++) {
            segment.points.insert(
                segment.points.end(),
                rowGroups[row].begin(),
                rowGroups[row].end()
                );
        }

        if (!segment.points.empty()) {
            segments.push_back(segment);
        }
    }

    return segments;
}

// Modyfikacja głównej funkcji detectMultiplePlanes
std::vector<MultiPlaneDetector::Plane> MultiPlaneDetector::detectMultiplePlanes(const QList<QList<int>>& sensorData) {
    std::vector<Plane> detectedPlanes;
    const int MAX_OUTLIERS = 5;
    const int MAX_ITERATIONS = 20;
    const double THRESHOLD_FACTOR = 2.5;

    if (!isValidSensorData(sensorData)) {
        qDebug() << "Invalid sensor data";
        return detectedPlanes;
    }

    // Preprocess data
    QList<QList<int>> processedData = Draw::preprocessSensorData(sensorData, 3);

    // Calculate initial coordinates
    std::vector<Point3D> coords = calculateCoordinates(processedData, rotatedAnglesX, rotatedAnglesZ);
    std::vector<DataSegment> segments = analyzeTrends(coords);

    // Dla każdego segmentu wykonaj dopasowanie płaszczyzny
    for (const auto& segment : segments) {
        std::vector<Point3D> current_coords = segment.points;

        if (current_coords.empty()) continue;

        int iteration = 0;
        while (iteration < MAX_ITERATIONS && !current_coords.empty()) {
            // Fit plane
            double a, b, c, d;
            fitPlane(current_coords, a, b, c, d);

            // Identify outliers
            std::vector<size_t> outlier_indices = identifyOutliers(current_coords, a, b, c, d, THRESHOLD_FACTOR);

            if (outlier_indices.empty()) {
                qDebug() << "No outliers detected in iteration" << iteration;

                // Create final plane
                Plane plane;
                plane.params.a = a;
                plane.params.b = b;
                plane.params.c = c;
                plane.params.d = d;

                // Obliczamy kąt względem płaszczyzny XY (kąt z normalną płaszczyzny XY [0,0,1])
                double dot_product = c;  // dot product z [0,0,1]
                double norm = std::sqrt(a*a + b*b + c*c);
                double angle = std::acos(std::abs(dot_product) / norm) * 180.0 / M_PI;

                // Określamy znak kąta na podstawie trendu wartości
                bool is_increasing = isIncreasingTrend(current_coords);
                plane.isIncreasingTrend = is_increasing;

                // Kąt azymutu to kąt względem płaszczyzny XY ze znakiem określonym przez trend
                plane.params.azimuth_angle = is_increasing ? angle : -angle;

                // Wyświetlamy dodatkowe informacje debugowe
                qDebug() << "Trend:" << (is_increasing ? "rosnący" : "malejący");
                qDebug() << "Kąt azymutu:" << plane.params.azimuth_angle;
                qDebug() << "Wektor normalny:" << a << b << c;

                // Klasyfikacja typu płaszczyzny na podstawie kąta z osią Z
                double normalized_c = std::abs(c/norm);
                if (normalized_c < 0.3) { // Mały kąt z osią Z oznacza płaszczyznę poziomą
                    plane.type = PlaneType::HORIZONTAL;
                    qDebug() << "Typ płaszczyzny: Pozioma";
                } else {
                    plane.type = PlaneType::VERTICAL;
                    qDebug() << "Typ płaszczyzny: Pionowa"
                             << "- wartości" << (is_increasing ? "rosną" : "maleją")
                             << "w kierunku płaszczyzny XY";
                }

                // Obliczanie kąta elewacji względem płaszczyzny XZ
                double projXZ = std::sqrt(a*a + c*c); // Projekcja na płaszczyznę XZ
                plane.params.elevation_angle = std::atan2(std::abs(b), projXZ) * 180.0 / M_PI;


                // Store used points
                for (const auto& p : current_coords) {
                    plane.rowsUsed.insert(p.row);
                    plane.colsUsed.insert(p.col);
                }

                // Calculate mean residual
                double total_residual = 0;
                for (const auto& p : current_coords) {
                    total_residual += std::abs(a * p.x + b * p.y + c * p.z + d);
                }
                plane.meanResidual = total_residual / current_coords.size();

                detectedPlanes.push_back(plane);
                break;
            }

            // Remove outliers and continue iteration
            std::vector<Point3D> new_coords;
            new_coords.reserve(current_coords.size() - outlier_indices.size());
            for (size_t i = 0; i < current_coords.size(); i++) {
                if (std::find(outlier_indices.begin(), outlier_indices.end(), i) == outlier_indices.end()) {
                    new_coords.push_back(current_coords[i]);
                }
            }
            current_coords = new_coords;
            iteration++;
        }
    }

    return detectedPlanes;
}
