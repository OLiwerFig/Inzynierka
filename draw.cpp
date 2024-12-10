#include "draw.h"
#include "multiplanedetector.h"
#include "ui_mainwindow.h"
#include <QString>
#include <cmath>
#include <QDebug>
#include <QGraphicsTextItem>
#include <QPen>
#include <QBrush>
#include <algorithm>

// Definicja stałej PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



static double rotatedAnglesZ[8][8] = {
    {-17.285, -18.134, -18.765, -19.104, -19.104, -18.765, -18.134, -17.285},
    {-12.732, -13.401, -13.904, -14.174, -14.174, -13.904, -13.401, -12.732},
    {-7.806, -8.236, -8.561, -8.737, -8.737, -8.561, -8.236, -7.806},
    {-2.631, -2.779, -2.892, -2.953, -2.953, -2.892, -2.779, -2.631},
    {2.631, 2.779, 2.892, 2.953, 2.953, 2.892, 2.779, 2.631},
    {7.806, 8.236, 8.561, 8.737, 8.737, 8.561, 8.236, 7.806},
    {12.732, 13.401, 13.904, 14.174, 14.174, 13.904, 13.401, 12.732},
    {17.285, 18.134, 18.765, 19.104, 19.104, 18.765, 18.134, 17.285}
};

static double rotatedAnglesX[8][8] = {
    {18.15, 18.63, 19.04, 19.9,  19.9,  19.04, 18.63, 18.15},
    {13.26, 13.65, 13.92, 14.5,  14.5,  13.92, 13.65, 13.26},
    {8.04,  8.34,  8.57,  8.82,  8.82,  8.57,  8.34,  8.04},
    {2.79,  2.86,  2.93,  2.96,  2.96,  2.93,  2.86,  2.79},
    {-2.79, -2.86, -2.93, -2.96, -2.96, -2.93, -2.86, -2.79},
    {-8.04, -8.34, -8.57, -8.82, -8.82, -8.57, -8.34, -8.04},
    {-13.26,-13.65,-13.92,-14.5, -14.5, -13.92,-13.65,-13.26},
    {-18.15,-18.63,-19.04,-19.9, -19.9, -19.04,-18.63,-18.15}
};

// Preprocesowanie danych czujnika: usuwanie outliers
QList<QList<int>> Draw::preprocessSensorData(const QList<QList<int>> &sensorData, int maxOutliers) {
    QList<QList<int>> data = sensorData;
    std::vector<std::pair<int, int>> outlierIndices;

    for (int i = 0; i < data.size(); ++i) {
        for (int j = 0; j < data[i].size(); ++j) {
            int current_value = data[i][j];
            std::vector<int> neighbors;

            for (int di = -1; di <=1; ++di) {
                for (int dj = -1; dj <=1; ++dj) {
                    if (di ==0 && dj ==0) continue;
                    int ni = i + di;
                    int nj = j + dj;
                    if (ni >=0 && ni < data.size() && nj >=0 && nj < data[i].size()) {
                        neighbors.push_back(data[ni][nj]);
                    }
                }
            }

            if (!neighbors.empty()) {
                double neighbor_mean = std::accumulate(neighbors.begin(), neighbors.end(), 0.0) / neighbors.size();
                if (current_value > 5 * neighbor_mean) {
                    outlierIndices.emplace_back(i, j);
                    data[i][j] = static_cast<int>(neighbor_mean); // Zastąpienie średnią
                }
            }
        }
    }

    if (outlierIndices.size() > static_cast<size_t>(maxOutliers)) {
        outlierIndices.erase(outlierIndices.begin() + maxOutliers, outlierIndices.end());
        qDebug() << "Warning: More than" << maxOutliers << "extreme outliers detected. Only correcting the first" << maxOutliers << ".";
    }

    return data;
}

QList<QList<int>> Draw::rotate90Right(const QList<QList<int>> &original) {
    if (original.isEmpty()) {
        qDebug() << "Empty input data in rotate90Right";
        return original;
    }

    QList<QList<int>> rotated(8, QList<int>(8,0));
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            rotated[i][j] = original[7-j][i];
        }
    }
    return rotated;
}

std::vector<Draw::PointInfo> Draw::calculateCoordinatesWithIndices(const QList<QList<int>>& sensorData) {
    std::vector<Draw::PointInfo> points;
    points.reserve(64);

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            double value = sensorData[i][j];
            double elevRad = MultiPlaneDetector::rotatedAnglesX[i][j] * M_PI / 180.0;
            double azimRad = MultiPlaneDetector::rotatedAnglesZ[i][j] * M_PI / 180.0;

            Draw::PointInfo p;
            p.row = i;
            p.col = j;
            p.x = value * std::cos(elevRad) * std::sin(azimRad);
            p.y = value * std::sin(elevRad);
            p.z = value * std::cos(elevRad) * std::cos(azimRad);

            points.push_back(p);
        }
    }
    return points;
}


void Draw::updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData, const std::vector<MultiPlaneDetector::Plane> &planes) {
    if (sensorData.size() != 8) {
        qDebug() << "Błąd: Nieprawidłowa liczba wierszy:" << sensorData.size();
        return;
    }

    for (const auto &row : sensorData) {
        if (row.size() != 8) {
            qDebug() << "Błąd: Nieprawidłowa liczba kolumn:" << row.size();
            return;
        }
    }

    scene->clear();
    QRectF viewRect = ui->graphicsView->viewport()->rect();
    float increasedWidth = viewRect.width() * 1.10f;
    float increasedHeight = viewRect.height() * 1.10f;
    float squareSizeX = increasedWidth / 8.0f;
    float squareSizeY = increasedHeight / 8.0f;

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            int value = sensorData[i][j];
            QColor color;
            if (value <= 400) {
                int red = qMin(255, value * 255 / 400);
                color = QColor(red, 0, 0);
            } else {
                int green = qMin(255, (value - 400) * 255 / 600);
                int red = 255 - green;
                color = QColor(red, green, 0);
            }

            scene->addRect(j * squareSizeX, i * squareSizeY, squareSizeX, squareSizeY,
                           QPen(Qt::black), QBrush(color));
            int fontSize = static_cast<int>(squareSizeX / 4);
            QGraphicsTextItem *text = scene->addText(QString::number(value),
                                                     QFont("Arial", fontSize, QFont::Bold));
            text->setDefaultTextColor(Qt::white);
            text->setPos(j * squareSizeX + squareSizeX / 4.0f,
                         i * squareSizeY + squareSizeY / 4.0f);
        }
    }

    ui->graphicsView->setSceneRect(0, 0, increasedWidth, increasedHeight);

    updateLabels(ui, sensorData);
}

void Draw::initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem) {
    Q_UNUSED(gridPixmapItem);
    scene = new QGraphicsScene(ui->graphicsView);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    scene->addLine(-150, 0, 150, 0, QPen(Qt::black));
    scene->addLine(0, -150, 0, 150, QPen(Qt::black));
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

void Draw::updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData) {
    qDebug() << "Rozpoczynam updateLabels";
    qDebug() << "Rozmiar danych wejściowych:" << sensorData.size();

    std::vector<MultiPlaneDetector::Plane> planes = MultiPlaneDetector::detectMultiplePlanes(sensorData);
    qDebug() << "Liczba wykrytych płaszczyzn:" << planes.size();

    if (planes.empty()) {
        qDebug() << "Brak wykrytych płaszczyzn";
        ui->AngleLabel->setText("Wykryto 0 płaszczyzn");
        ui->SurfaceLabel->setText("Brak płaszczyzn do wyświetlenia");
        return;
    }

    // Wyświetl dane pierwszej płaszczyzny dla debugowania
    if (!planes.empty()) {
        const auto &firstPlane = planes[0];
        qDebug() << "Parametry pierwszej płaszczyzny:";
        qDebug() << "a:" << firstPlane.params.a;
        qDebug() << "b:" << firstPlane.params.b;
        qDebug() << "c:" << firstPlane.params.c;
        qDebug() << "d:" << firstPlane.params.d;
        qDebug() << "azimuth:" << firstPlane.params.azimuth_angle;
        qDebug() << "elevation:" << firstPlane.params.elevation_angle;
        qDebug() << "trend:" << firstPlane.isIncreasingTrend;
    }

    QString description;
    for (size_t i = 0; i < planes.size(); i++) {
        qDebug() << "Przetwarzam płaszczyznę" << i;
        const auto &plane = planes[i];
        double azimuth = plane.params.azimuth_angle;
        double elevation = plane.params.elevation_angle;
        QString directionAzimuth = azimuth < 0 ? "LEWO" : "PRAWO";
        QString directionElevation = elevation < 0 ? "DÓŁ" : "GÓRA";
        QString trend = plane.isIncreasingTrend ? "ROSNĄCY" : "MALEJĄCY";

        description += QString("Płaszczyzna %1:\n").arg(i+1);
        description += QString("  Równanie płaszczyzny: %1x + %2y + %3z + %4 = 0\n")
                           .arg(plane.params.a, 0, 'f', 4)
                           .arg(plane.params.b, 0, 'f', 4)
                           .arg(plane.params.c, 0, 'f', 4)
                           .arg(plane.params.d, 0, 'f', 4);
        description += QString("  Kąt azymutu: %1° (%2)\n").arg(azimuth, 0, 'f', 1).arg(directionAzimuth);
        description += QString("  Kąt elewacji: %1° (%2)\n").arg(elevation, 0, 'f', 1).arg(directionElevation);
        description += QString("  Trend wartości: %1\n").arg(trend);
        description += QString("  Średnie odchylenie: %1 mm\n").arg(plane.meanResidual,0,'f',2);

        if (!plane.rowsUsed.empty() && !plane.colsUsed.empty()) {
            description += "  Wykorzystane wiersze: ";
            for (auto r : plane.rowsUsed)
                description += QString::number(r) + " ";
            description += "\n";
            description += "  Wykorzystane kolumny: ";
            for (auto c : plane.colsUsed)
                description += QString::number(c) + " ";
            description += "\n";
        }
        description += "\n";
    }

    qDebug() << "Opis do wyświetlenia:" << description;
    ui->AngleLabel->setText(QString("Wykryto %1 płaszczyzn").arg(planes.size()));
    ui->SurfaceLabel->setText(description);
    qDebug() << "Zakończono updateLabels";
}
