#ifndef DRAW_H
#define DRAW_H

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <Eigen/Dense>
#include <QList>
#include "ui_mainwindow.h"

// Dodajemy deklaracje Qt Data Visualization
#include <QtDataVisualization/Q3DScatter>
#include <QtDataVisualization/Q3DSurface>
#include <QtDataVisualization/QScatter3DSeries>
#include <QtDataVisualization/QSurface3DSeries>
#include <QtDataVisualization/QSurfaceDataProxy>
#include <QtDataVisualization/QScatterDataProxy>

QT_BEGIN_NAMESPACE
class Q3DScatter;
class Q3DSurface;
QT_END_NAMESPACE

class Draw {
public:
    // Inicjalizacja sceny 2D
    static void initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem);

    // Aktualizacja danych i wyświetlanie macierzy 8x8 w 2D
    static void updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData);

    // Aktualizacja etykiet (kąty, płaszczyzna)
    static void updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);

    // Debug
    static void printMatrixToQDebug(const Eigen::MatrixXd &matrix);

    // Funkcja do wyświetlenia widoku 3D (drugi sposób wizualizacji)
    static void show3DView(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);

private:
    struct PlaneParams {
        double a,b,c,d;
    };

    struct FitResult {
        PlaneParams plane;
        Eigen::MatrixXd inlierCoords;
        Eigen::VectorXd residuals;
        QList<QList<int>> processedData;
    };

    static QList<QList<int>> rotate90Right(const QList<QList<int>> &original);
    static QList<QList<int>> preprocessSensorData(const QList<QList<int>> &sensorData, int max_outliers=3);
    static Eigen::MatrixXd calculateCoordinates(const QList<QList<int>> &sensorData);
    static PlaneParams fitPlane(const Eigen::MatrixXd &coords);
    static QList<int> identifyOutliers(const Eigen::MatrixXd &coords, const PlaneParams &plane, double threshold_factor=2.5);
    static FitResult iterativePlaneFitting(const QList<QList<int>> &sensorData, int max_outliers=5, int max_iterations=20, double threshold_factor=2.5);
};

#endif // DRAW_H
