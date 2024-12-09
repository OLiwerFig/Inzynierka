#ifndef MULTIPLANEDETECTOR_H
#define MULTIPLANEDETECTOR_H

#include <QList>
#include <vector>

class MultiPlaneDetector {
public:
    struct Plane {
        struct Params {
            double a, b, c, d;
            double azimuth_angle;
        } params;
        double meanResidual;
        bool isLeftPlane;

        // Wektory z indeksami w obróconym układzie
        std::vector<int> rowsUsed;
        std::vector<int> colsUsed;
    };

    static bool isValidSensorData(const QList<QList<int>>& sensorData);

    // Wykrywa płaszczyzny w danych obróconych o 90°
    static std::vector<Plane> detectMultiplePlanes(const QList<QList<int>>& sensorData);
};

#endif // MULTIPLANEDETECTOR_H
