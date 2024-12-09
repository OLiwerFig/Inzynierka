#include "draw.h"
#include "multiplanedetector.h"
#include "ui_mainwindow.h"
#include <QString>
#include <cmath>
#include <QDebug>
#include <QGraphicsTextItem>
#include <QPen>
#include <QBrush>

#define PI 3.14159265

// Tablice kątów (elewacja i azymut) w stopniach
static double rotatedAnglesZ[8][8] = {
    {-18.15, -13.26, -8.04, -2.79,  2.79,  8.04, 13.26, 18.15},
    {-18.63, -13.65, -8.34, -2.86,  2.86,  8.34, 13.65, 18.63},
    {-19.04, -13.92, -8.57, -2.93,  2.93,  8.57, 13.92, 19.04},
    {-19.9,  -14.5,  -8.82, -2.96,  2.96,  8.82, 14.5,  19.9 },
    {-19.9,  -14.5,  -8.82, -2.96,  2.96,  8.82, 14.5,  19.9 },
    {-19.04, -13.92, -8.57, -2.93,  2.93,  8.57, 13.92, 19.04},
    {-18.63, -13.65, -8.34, -2.86,  2.86,  8.34, 13.65, 18.63},
    {-18.15, -13.26, -8.04, -2.79,  2.79,  8.04, 13.26, 18.15}
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

std::vector<Draw::PointInfo> Draw::calculateCoordinatesWithIndices(const QList<QList<int>> &sensorData) {
    if (sensorData.isEmpty()) {
        throw std::runtime_error("Empty sensor data");
    }

    int rows = sensorData.size();
    int cols = sensorData[0].size();

    if (rows != 8 || cols !=8) {
        throw std::runtime_error("Data must be 8x8");
    }

    std::vector<PointInfo> points;
    points.reserve(rows * cols);

    // Zakładamy, że tablice kątów dotyczą pozycji w macierzy po obrocie
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            double d_value = sensorData[i][j];
            double elev = rotatedAnglesX[i][j] * PI / 180.0;
            double azim = rotatedAnglesZ[i][j] * PI / 180.0;

            PointInfo p;
            p.row = i;
            p.col = j;
            p.x = d_value * std::cos(elev) * std::sin(azim);
            p.y = d_value * std::sin(elev);
            p.z = d_value * std::cos(elev) * std::cos(azim);

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
    std::vector<MultiPlaneDetector::Plane> planes = MultiPlaneDetector::detectMultiplePlanes(sensorData);
    if (planes.empty()) {
        ui->AngleLabel->setText("Wykryto 0 płaszczyzn");
        ui->SurfaceLabel->setText("Brak płaszczyzn do wyświetlenia");
        return;
    }

    ui->AngleLabel->setText(QString("Wykryto %1 płaszczyzn").arg(planes.size()));

    QString description;
    for (size_t i = 0; i < planes.size(); i++) {
        const auto &plane = planes[i];
        double angle = plane.params.azimuth_angle;
        QString direction = std::abs(angle) < 10.0 ? "PRZÓD" :
                                (angle < 0 ? "LEWO" : "PRAWO");

        description += QString("Płaszczyzna %1:\n").arg(i+1);
        description += QString("  Kierunek: %1 (kąt: %2°)\n").arg(direction).arg(angle,0,'f',1);
        description += QString("  Średnie odchylenie: %1 mm\n").arg(plane.meanResidual,0,'f',2);

        // Wyświetlamy indeksy w obróconym układzie, który traktujemy jako "oryginalny".
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

    ui->SurfaceLabel->setText(description);
}

