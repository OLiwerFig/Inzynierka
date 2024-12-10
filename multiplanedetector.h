#ifndef MULTIPLANEDETECTOR_H
#define MULTIPLANEDETECTOR_H

#include <QList>
#include <vector>
#include <set>

class MultiPlaneDetector {
public:
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
        bool isIncreasingTrend;  // Dodane pole dla trendu
    };

    static bool isValidSensorData(const QList<QList<int>>& sensorData);
    static std::vector<Plane> detectMultiplePlanes(const QList<QList<int>>& sensorData);

    // Tablice kątów jako static constexpr i public, żeby były dostępne w Draw
    static constexpr double rotatedAnglesX[8][8] = {
        {18.15, 18.63, 19.04, 19.9,  19.9,  19.04, 18.63, 18.15},
        {13.26, 13.65, 13.92, 14.5,  14.5,  13.92, 13.65, 13.26},
        {8.04,  8.34,  8.57,  8.82,  8.82,  8.57,  8.34,  8.04},
        {2.79,  2.86,  2.93,  2.96,  2.96,  2.93,  2.86,  2.79},
        {-2.79, -2.86, -2.93, -2.96, -2.96, -2.93, -2.86, -2.79},
        {-8.04, -8.34, -8.57, -8.82, -8.82, -8.57, -8.34, -8.04},
        {-13.26,-13.65,-13.92,-14.5, -14.5, -13.92,-13.65,-13.26},
        {-18.15,-18.63,-19.04,-19.9, -19.9, -19.04,-18.63,-18.15}
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
};

#endif // MULTIPLANEDETECTOR_H
