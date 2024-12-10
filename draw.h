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

    // Funkcja preprocesująca dane czujnika
    static QList<QList<int>> preprocessSensorData(const QList<QList<int>> &sensorData, int maxOutliers);

    // Funkcja obracająca dane o 90 stopni w prawo
    static QList<QList<int>> rotate90Right(const QList<QList<int>> &original);

    // Konwersja (po obrocie) do chmury punktów 3D
    static std::vector<PointInfo> calculateCoordinatesWithIndices(const QList<QList<int>> &sensorData);

    // Aktualizacja danych czujnika i wizualizacja
    static void updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData, const std::vector<MultiPlaneDetector::Plane> &planes);

    // Inicjalizacja sceny graficznej
    static void initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem);

    // Aktualizacja etykiet w UI
    static void updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);
};

#endif // DRAW_H
