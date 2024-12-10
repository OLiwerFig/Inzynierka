#ifndef DRAW_H
#define DRAW_H

#include <QList>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <vector>
#include "multiplanedetector.h"

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

    static QList<QList<int>> preprocessSensorData(const QList<QList<int>> &sensorData, int maxOutliers);
    static QList<QList<int>> rotate90Right(const QList<QList<int>> &original);
    static std::vector<PointInfo> calculateCoordinatesWithIndices(const QList<QList<int>> &sensorData);
    static void updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene,
                                 const QList<QList<int>> &sensorData,
                                 const std::vector<MultiPlaneDetector::Plane> &planes);
    static void initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene,
                                        QGraphicsPixmapItem *&gridPixmapItem);
    static void updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);
};

#endif // DRAW_H
