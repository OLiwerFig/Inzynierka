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

struct Point3D {
    double x, y, z;
    int row, col;
};

static std::vector<Point3D> calculateCoordinates(const QList<QList<int>>& sensorData,
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

static void fitPlane(const std::vector<Point3D>& coords, double& a, double& b, double& c, double& d) {
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

static std::vector<size_t> identifyOutliers(const std::vector<Point3D>& coords,
                                            double a, double b, double c, double d,
                                            double threshold_factor = 2.5) {
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
    std::vector<Point3D> current_coords = coords;

    int iteration = 0;
    int total_outliers = 0;

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

            // Calculate angles
            plane.params.elevation_angle = std::atan2(std::abs(a),
                                                      std::sqrt(b * b + c * c)) * 180.0 / M_PI;
            plane.params.azimuth_angle = std::atan2(std::abs(c),
                                                    std::sqrt(a * a + b * b)) * 180.0 / M_PI;

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

        // Check outlier limit
        int num_new_outliers = outlier_indices.size();
        total_outliers += num_new_outliers;

        if (total_outliers > MAX_OUTLIERS) {
            int num_allowed = MAX_OUTLIERS - (total_outliers - num_new_outliers);
            qDebug() << "Total outliers exceeded" << MAX_OUTLIERS << ". Only excluding" << num_allowed << "more outlier(s).";
            outlier_indices.resize(num_allowed);
        }

        // Remove outliers
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

    if (iteration == MAX_ITERATIONS) {
        qDebug() << "Maximum iterations reached. Some outliers may remain.";
    }

    return detectedPlanes;
}
