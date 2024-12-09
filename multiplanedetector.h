
// multiplane_detector.h
#ifndef MULTIPLANE_DETECTOR_H
#define MULTIPLANE_DETECTOR_H
#include <Eigen/Dense>
#include <vector>
#include <QString>
#include "draw.h"
#include <QMap>

    class MultiPlaneDetector {
private:
    struct SegmentAnalysis {
        int startIdx;
        int endIdx;
        double meanValue;
        bool isIncreasing;
        int pointCount;
    };

    struct PlaneSegment {
        int startRow;
        int endRow;
        bool isIncreasing;
    };

public:
    struct Plane {
        Draw::PlaneParams params;
        int startRow;
        int endRow;
        double meanResidual;
        bool isLeftPlane;  // Dodane pole
        std::vector<int> cornerRows;
        std::vector<int> cornerCols;
    };

    // Metody publiczne
    static std::vector<Plane> detectMultiplePlanes(const QList<QList<int>>& sensorData);
    static QString generatePlanesDescription(const std::vector<Plane>& planes);

private:
    // Metody prywatne
    static std::vector<SegmentAnalysis> analyzeRowTrend(const QList<QList<int>>& data, int row);
    static std::vector<PlaneSegment> findPlanesInSegment(const QList<QList<int>>& data,
                                                         int startRow, int endRow);
    static std::vector<PlaneSegment> analyzeColumnTrend(const QList<QList<int>>& data, int col);
    static std::vector<PlaneSegment> analyzeRowData(const QList<QList<int>>& data, int row);
    static std::vector<int> findPlaneBoundaries(const QList<QList<int>>& data);
    static QList<QList<int>> extractPlaneData(const QList<QList<int>>& data,
                                              int startRow, int endRow);
    static void findCorners(const QList<QList<int>>& data, std::vector<Plane>& planes);
    static bool isValidSensorData(const QList<QList<int>>& sensorData);
};

#endif  // MULTIPLANE_DETECTOR_H
