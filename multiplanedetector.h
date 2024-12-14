#ifndef MULTIPLANEDETECTOR_H
#define MULTIPLANEDETECTOR_H

#include <QList>
#include <vector>
#include <set>
#include <Eigen/Dense>

// Przeniesienie Point3D do nagłówka
struct Point3D {
    double x, y, z;
    int row, col;
};

class MultiPlaneDetector {
public:
    enum class PlaneType {
        VERTICAL,
        HORIZONTAL
    };

    struct PlaneParameters {
        double a, b, c, d;
        double elevation_angle;
        double azimuth_angle;
    };

    struct Plane {
        PlaneParameters params;
        std::set<int> rowsUsed;
        std::set<int> colsUsed;
        double meanResidual;
        bool isIncreasingTrend;
        PlaneType type;  // Nowe pole
    };




    struct DataSegment {
        std::vector<Point3D> points;
        int startRow;
        int endRow;
        int startCol;  // Nowe pole
        int endCol;    // Nowe pole
        bool isIncreasing;
    };

    static bool isValidSensorData(const QList<QList<int>>& sensorData);
    static std::vector<Plane> detectMultiplePlanes(const QList<QList<int>>& sensorData);

private:
    static constexpr double rotatedAnglesX[8][8] = {
        {-18.161, -13.423, -8.250, -2.784, 2.784, 8.250, 13.423, 18.161},
        {-18.624, -13.791, -8.488, -2.867, 2.867, 8.488, 13.791, 18.624},
        {-18.954, -14.054, -8.659, -2.926, 2.926, 8.659, 14.054, 18.954},
        {-19.125, -14.192, -8.748, -2.957, 2.957, 8.748, 14.192, 19.125},
        {-19.125, -14.192, -8.748, -2.957, 2.957, 8.748, 14.192, 19.125},
        {-18.954, -14.054, -8.659, -2.926, 2.926, 8.659, 14.054, 18.954},
        {-18.624, -13.791, -8.488, -2.867, 2.867, 8.488, 13.791, 18.624},
        {-18.161, -13.423, -8.250, -2.784, 2.784, 8.250, 13.423, 18.161}
    };

    static constexpr double rotatedAnglesZ[8][8] = {
        {-17.285, -18.134, -18.765, -19.104, -19.104, -18.765, -18.134, -17.285},
        {-12.732, -13.401, -13.904, -14.174, -14.174, -13.904, -13.401, -12.732},
        {-7.806, -8.236, -8.561, -8.737, -8.737, -8.561, -8.236, -7.806},
        {-2.631, -2.779, -2.892, -2.953, -2.953, -2.892, -2.779, -2.631},
        {2.631, 2.779, 2.892, 2.953, 2.953, 2.892, 2.779, 2.631},
        {7.806, 8.236, 8.561, 8.737, 8.737, 8.561, 8.236, 7.806},
        {12.732, 13.401, 13.904, 14.174, 14.174, 13.904, 13.401, 12.732},
        {17.285, 18.134, 18.765, 19.104, 19.104, 18.765, 18.134, 17.285}
    };

    // Deklaracje funkcji pomocniczych
    static std::vector<Point3D> calculateCoordinates(const QList<QList<int>>& sensorData,
                                                     const double elevationAngles[8][8],
                                                     const double azimuthAngles[8][8]);
    static void fitPlane(const std::vector<Point3D>& coords, double& a, double& b, double& c, double& d);
    static std::vector<size_t> identifyOutliers(const std::vector<Point3D>& coords,
                                                double a, double b, double c, double d,
                                                double threshold_factor);
    static bool isIncreasingTrend(const std::vector<Point3D>& coords);
    static std::vector<DataSegment> analyzeTrends(const std::vector<Point3D>& coords);
    static std::vector<DataSegment> analyzeColumnTrends(const std::vector<Point3D>& coords);
};


#endif
