#ifndef DRAW_H
#define DRAW_H

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPen>
#include <QBrush>
#include <QList>
#include <QPointF>
#include <QGraphicsEllipseItem>
#include "ui_mainwindow.h"
#include <Eigen/Dense>



class Draw {
public:
    // Funkcja inicjalizująca scenę graficzną
    static void initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem);

    // Funkcja aktualizująca dane czujnika i rysująca macierz 8x8
    static void updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData);

    // Funkcja aktualizująca etykiety QLabel dla kąta i powierzchni
    static void updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData);

    // Funkcja obliczająca kąt patrzenia na płaszczyznę na podstawie danych czujnika
    static double calculateViewingAngle(const QList<QList<int>> &sensorData);

    // Funkcja obliczająca współczynniki płaszczyzny
    static Eigen::Vector4d calculatePlaneCoefficients(const QList<QList<int>> &sensorData);

    static void printMatrixToQDebug(const Eigen::MatrixXd& matrix);
};

#endif // DRAW_H
