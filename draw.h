#ifndef DRAW_H
#define DRAW_H

#include <QList>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <Eigen/Dense>
#include <vector>
#include "multiplanedetector.h"
class Ui_MainWindow; // Forward declaration

namespace Ui {
class MainWindow;
}

class Draw {
public:
    struct PointInfo {
        int row;
        int col;
        double x;
        double y;
        double z;
    };

    struct PlaneParams {
        double a;
        double b;
        double c;
        double d;
        double azimuth_angle;
    };

    struct FitResult {
        PlaneParams plane;
        Eigen::MatrixXd inlierCoords;
        Eigen::VectorXd residuals;
        QList<QList<int>> processedData;
    };

    static QList<QList<int>> rotate90Right(const QList<QList<int>> &original);

    // Konwersja (po obrocie) do chmury punktów 3D
    static std::vector<PointInfo> calculateCoordinatesWithIndices(const QList<QList<int>> &sensorData);

    static void updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData, const std::vector<MultiPlaneDetector::Plane> &planes);

    static void initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem);

    static void updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);
};

#endif // DRAW_H
