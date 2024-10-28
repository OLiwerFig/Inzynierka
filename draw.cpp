#include "draw.h"
#include <QString>
#include <cmath>
#include <Eigen/Dense>
#include <QDebug>

#define PI 3.14159265

// Tabela kątów dla każdego piksela (dla osi X - poziomo)
double anglesX[8][8] = {
    {22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5, 22.5},
    {15.2, 15.2, 15.2, 15.2, 15.2, 15.2, 15.2, 15.2},
    {7.7, 7.7, 7.7, 7.7, 7.7, 7.7, 7.7, 7.7},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5},
    {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5},
    {-7.7, -7.7, -7.7, -7.7, -7.7, -7.7, -7.7, -7.7},
    {-15.2, -15.2, -15.2, -15.2, -15.2, -15.2, -15.2, -15.2},
    {-22.5, -22.5, -22.5, -22.5, -22.5, -22.5, -22.5, -22.5}
};

// Tabela kątów dla każdego piksela (dla osi Y - pionowo)
double rotatedAnglesY[8][8] = {
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5},
    {22.5, 15.2, 7.7, 0.5, -0.5, -7.7, -15.2, -22.5}
};

void Draw::initializeGraphicsScene(Ui::MainWindow *ui, QGraphicsScene *&scene, QGraphicsPixmapItem *&gridPixmapItem) {
    (void)gridPixmapItem;
    scene = new QGraphicsScene(ui->graphicsView);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    scene->addLine(-150, 0, 150, 0, QPen(Qt::black));
    scene->addLine(0, -150, 0, 150, QPen(Qt::black));
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

void Draw::printMatrixToQDebug(const Eigen::MatrixXd& matrix) {
    for (int i = 0; i < matrix.rows(); ++i) {
        QString row = "";
        for (int j = 0; j < matrix.cols(); ++j) {
            row += QString::number(matrix(i, j)) + " ";
        }
        qDebug() << row;
    }
}

Eigen::Vector4d Draw::calculatePlaneCoefficients(const QList<QList<int>> &sensorData) {
    int numPoints = sensorData.size() * sensorData[0].size();
    Eigen::MatrixXd A(numPoints, 3);  // macierz z kolumnami x, y, z
    Eigen::VectorXd B(numPoints);     // wektor wynikowy dla równania

    int index = 0;
    for (int i = 0; i < sensorData.size(); ++i) {
        for (int j = 0; j < sensorData[i].size(); ++j) {
            // Używamy wartości z tabeli kątów dla X i Y
            double angleX = anglesX[i][j];
            double angleY = rotatedAnglesY[i][j];

            // Obliczamy współrzędne x, y na podstawie kąta
            double z = sensorData[i][j]; // Z to wartość z czujnika
            double x = z * tan(angleX * PI / 180.0); // x jest proporcjonalny do kąta X
            double y = z * tan(angleY * PI / 180.0); // y jest proporcjonalny do kąta Y

            A(index, 0) = x;
            A(index, 1) = y;
            A(index, 2) = 1;  // Stała wartość
            B(index) = z;      // Wartość z w wektorze B
            index++;
        }
    }

    // Rozwiązywanie A * [a, b, d]^T = B
    Eigen::Vector3d planeCoefficients = (A.transpose() * A).ldlt().solve(A.transpose() * B);

    // Konstruowanie wektora [a, b, c, d]
    double a = planeCoefficients(0);
    double b = planeCoefficients(1);
    double d = planeCoefficients(2);
    double c = -1;  // Zakładamy, że c jest -1 dla standardowego równania

    Eigen::Vector4d result(a, b, c, d);
    return result;
}

void Draw::updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData) {
    Eigen::Vector4d planeCoefficients = Draw::calculatePlaneCoefficients(sensorData);

    double a = planeCoefficients(0);
    double b = planeCoefficients(1);
    double c = planeCoefficients(2);
    double d = planeCoefficients(3);

    double normFactor = sqrt(a * a + b * b + c * c);
    if (normFactor != 0) {
        // Normalizacja współczynników bez zmiany znaku
        a /= normFactor;
        b /= normFactor;
        c /= normFactor;
    } else {
        a = 0;
        b = 0;
        c = 1; // Domyślne ustawienie dla pionowego wektora
    }

    QString planeText = QString("Współczynniki płaszczyzny: a=%1, b=%2, c=%3, d=%4")
                            .arg(a)
                            .arg(b)
                            .arg(c)
                            .arg(d);
    ui->SurfaceLabel->setText(planeText);

    qDebug() << "Współczynnik A:" << a << "Współczynnik B:" << b << "Współczynnik C:" << c << "Współczynnik D:" << d;

    // Obliczamy kąty patrzenia względem osi X, Y i Z z poprawnym znakiem
    double viewingAngleX = atan2(a, sqrt(b * b + c * c)) * 180.0 / PI;
    double viewingAngleY = atan2(b, sqrt(a * a + c * c)) * 180.0 / PI;
    double viewingAngleZ = acos(c) * 180.0 / PI;  // Kąt względem osi Z

    QString angleText = QString("Kąt osi X: %1°, Y: %2°, Z: %3°")
                            .arg(viewingAngleX)
                            .arg(viewingAngleY)
                            .arg(viewingAngleZ);
    ui->AngleLabel->setText(angleText);
}

void Draw::updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData) {
    scene->clear();
    QRectF viewRect = ui->graphicsView->viewport()->rect();
    float increasedWidth = viewRect.width() * 1.10;
    float increasedHeight = viewRect.height() * 1.10;
    float squareSizeX = increasedWidth / 8;
    float squareSizeY = increasedHeight / 8;

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            int value = sensorData[i][j];
            QColor color;
            if (value <= 400) {
                int red = qMin(255, value * 255 / 400);
                int green = 0;
                int blue = 0;
                color = QColor(red, green, blue);
            } else {
                int green = qMin(255, (value - 400) * 255 / 600);
                int red = 255 - green;
                color = QColor(red, green, 0);
            }
            scene->addRect(j * squareSizeX, i * squareSizeY, squareSizeX, squareSizeY, QPen(Qt::black), QBrush(color));
            int fontSize = static_cast<int>(squareSizeX / 4);
            QGraphicsTextItem *text = scene->addText(QString::number(value), QFont("Arial", fontSize, QFont::Bold));
            text->setDefaultTextColor(Qt::white);
            text->setPos(j * squareSizeX + squareSizeX / 4, i * squareSizeY + squareSizeY / 4);
        }
    }
    ui->graphicsView->setSceneRect(0, 0, increasedWidth, increasedHeight);
    updateLabels(ui, sensorData);
}
