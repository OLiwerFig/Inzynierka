#include "draw.h"
#include <QString>
#include <cmath>
#include <Eigen/Dense>
#include <QDebug>
#include <QGraphicsTextItem>
#include <QPen>
#include <QBrush>
#include "multiplanedetector.h"

#define PI 3.14159265

// Tablice kątów podane przez użytkownika (elewacja i azymut w stopniach)
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

void Draw::printMatrixToQDebug(const Eigen::MatrixXd &matrix) {
    for (int i = 0; i < matrix.rows(); ++i) {
        QString row;
        for (int j = 0; j < matrix.cols(); ++j) {
            row += QString::number(matrix(i, j)) + " ";
        }
        qDebug() << row;
    }
}



static double neighborMean(const QList<QList<int>> &data, int i, int j) {
    double sum = 0.0;
    int count = 0;
    int rows = data.size();
    int cols = data[0].size();

    for (int di = -1; di <= 1; di++) {
        for (int dj = -1; dj <= 1; dj++) {
            if (di == 0 && dj == 0) continue;
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                sum += data[ni][nj];
                count++;
            }
        }
    }
    if (count > 0) return sum / count;
    return data[i][j];
}

Eigen::MatrixXd Draw::calculateCoordinates(const QList<QList<int>> &sensorData) {
    if (sensorData.isEmpty()) {
        throw std::runtime_error("Empty sensor data");
    }

    int rows = sensorData.size();
    int cols = sensorData[0].size();
    int total_points = rows * cols;

    qDebug() << "Calculating coordinates for" << rows << "x" << cols << "data";

    // Sprawdź czy każdy wiersz ma tę samą długość
    for (const auto& row : sensorData) {
        if (row.size() != cols) {
            throw std::runtime_error("Irregular data dimensions");
        }
    }

    Eigen::MatrixXd coords(total_points, 3);

    // Znajdź początkowy indeks dla tablicy kątów
    int startRow = 0;
    if (rows < 8) {
        // Jeśli mamy segment danych, ustal odpowiedni początek w tablicy kątów
        for (int i = 0; i < 8 - rows + 1; i++) {
            bool match = true;
            for (int j = 0; j < rows; j++) {
                for (int k = 0; k < cols; k++) {
                    if (k >= 8) continue;
                    double value1 = sensorData[j][k];
                    double value2 = rotatedAnglesX[i + j][k];
                    if (std::abs(value1 - value2) > 1000) { // Duża różnica wskazuje na brak dopasowania
                        match = false;
                        break;
                    }
                }
                if (!match) break;
            }
            if (match) {
                startRow = i;
                break;
            }
        }
    }

    int index = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Użyj odpowiednich indeksów dla tablicy kątów
            int angleRow = startRow + i;
            int angleCol = j;

            // Zabezpiecz przed wyjściem poza zakres tablicy kątów
            if (angleRow >= 8 || angleCol >= 8) {
                qDebug() << "Warning: Using boundary angles for point" << i << j;
                angleRow = std::min(angleRow, 7);
                angleCol = std::min(angleCol, 7);
            }

            double d_value = sensorData[i][j];
            double elev = rotatedAnglesX[angleRow][angleCol] * PI / 180.0;
            double azim = rotatedAnglesZ[angleRow][angleCol] * PI / 180.0;

            coords(index, 0) = d_value * std::cos(elev) * std::sin(azim);
            coords(index, 1) = d_value * std::sin(elev);
            coords(index, 2) = d_value * std::cos(elev) * std::cos(azim);
            index++;
        }
    }

    qDebug() << "Successfully created coordinate matrix with" << coords.rows() << "points";
    return coords;
}

QList<QList<int>> Draw::preprocessSensorData(const QList<QList<int>> &sensorData, int max_outliers) {
    QList<QList<int>> data = sensorData;
    QList<QPair<int,int>> outlierIndices;

    int rows = data.size();
    int cols = data[0].size();

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            double current_value = data[i][j];
            double nm = neighborMean(data, i, j);
            if (nm > 0 && current_value > 5.0 * nm) {
                outlierIndices.append(qMakePair(i,j));
            }
        }
    }

    if (outlierIndices.size() > max_outliers) {
        qDebug() << "Warning: More than" << max_outliers << "extreme outliers detected. Only correcting the first" << max_outliers;
        outlierIndices = outlierIndices.mid(0, max_outliers);
    }

    for (auto &idx : outlierIndices) {
        data[idx.first][idx.second] = (int)neighborMean(data, idx.first, idx.second);
    }

    return data;
}

Draw::PlaneParams Draw::fitPlane(const Eigen::MatrixXd &coords) {
    int Np = (int)coords.rows();
    Eigen::VectorXd x = coords.col(0);
    Eigen::VectorXd y = coords.col(1);
    Eigen::VectorXd z = coords.col(2);

    double S_xx = (x.array()*x.array()).sum();
    double S_xy = (x.array()*y.array()).sum();
    double S_xz = (x.array()*z.array()).sum();
    double S_yy = (y.array()*y.array()).sum();
    double S_yz = (y.array()*z.array()).sum();
    double S_zz = (z.array()*z.array()).sum();
    double S_x1 = x.sum();
    double S_y1 = y.sum();
    double S_z1 = z.sum();

    Eigen::Matrix3d M;
    M << S_xx, S_xy, S_xz,
        S_xy, S_yy, S_yz,
        S_xz, S_yz, S_zz;

    Eigen::Vector3d k(S_x1, S_y1, S_z1);
    Eigen::Matrix3d M_prime = M - (k * k.transpose()) / Np;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M_prime);
    Eigen::VectorXd eigvals = es.eigenvalues();
    Eigen::Matrix3d eigvecs = es.eigenvectors();

    int idx_min = 0;
    double min_val = eigvals(0);
    for (int i=1; i<3; i++) {
        if (eigvals(i) < min_val) {
            min_val = eigvals(i);
            idx_min = i;
        }
    }

    // Wektor normalny do płaszczyzny
    Eigen::Vector3d normal_vector = eigvecs.col(idx_min);

    // Normalizacja wektora normalnego
    double norm = normal_vector.norm();
    if (norm > 1e-12) {
        normal_vector /= norm;
    } else {
        normal_vector = Eigen::Vector3d(0, 0, 1);
    }

    // Współczynniki płaszczyzny
    double a = normal_vector(0);
    double b = normal_vector(1);
    double c = normal_vector(2);
    double d = -(a * S_x1 + b * S_y1 + c * S_z1) / Np;

    // Oblicz kąt azymutu (w płaszczyźnie XZ)
    // atan2 zwróci kąt w zakresie (-π, π)
    double azimuth_angle = std::atan2(a, c) * 180.0 / M_PI;

    // Zapisz kąt azymutu jako dodatkową informację
    qDebug() << "Fitted plane with azimuth angle:" << azimuth_angle << "degrees";
    qDebug() << "Normal vector:" << a << b << c;

    PlaneParams p{a,b,c,d, azimuth_angle};  // Dodaj kąt do struktury
    return p;
}

QList<int> Draw::identifyOutliers(const Eigen::MatrixXd &coords, const PlaneParams &plane, double threshold_factor) {
    int Np = (int)coords.rows();
    Eigen::VectorXd residuals(Np);
    for (int i = 0; i < Np; i++) {
        double rx=coords(i,0), ry=coords(i,1), rz=coords(i,2);
        double dist = std::abs(plane.a*rx + plane.b*ry + plane.c*rz + plane.d);
        residuals(i)=dist;
    }

    double mean_res = residuals.mean();
    double var_res = (residuals.array()-mean_res).square().mean();
    double std_res = sqrt(var_res);
    double threshold = mean_res + threshold_factor * std_res;

    QList<int> outliers;
    for (int i=0;i<Np;i++) {
        if (residuals(i)>threshold) outliers.append(i);
    }

    return outliers;
}

Draw::FitResult Draw::iterativePlaneFitting(const QList<QList<int>> &sensorData, int max_outliers, int max_iterations, double threshold_factor) {
    qDebug() << "Starting plane fitting with" << sensorData.size() << "rows and"
             << (sensorData.isEmpty() ? 0 : sensorData[0].size()) << "columns";

    // Sprawdź minimalne wymagania dla danych
    if (sensorData.size() < 4 || sensorData[0].size() < 4) {
        throw std::runtime_error("Insufficient data points for plane fitting");
    }

    // Preprocessing z walidacją
    QList<QList<int>> processed = preprocessSensorData(sensorData, 3);
    if (processed.size() < 4 || processed[0].size() < 4) {
        throw std::runtime_error("Insufficient valid data points after preprocessing");
    }

    try {
        Eigen::MatrixXd coords_all = calculateCoordinates(processed);
        if (coords_all.rows() < 4) {
            throw std::runtime_error("Not enough points for plane fitting");
        }

        Eigen::MatrixXd coords = coords_all;
        PlaneParams plane;
        Eigen::VectorXd residuals;
        int iteration = 0;
        int total_outliers = 0;

        while (iteration < max_iterations && coords.rows() >= 4) {
            qDebug() << "Iteration" << iteration << "with" << coords.rows() << "points";

            try {
                plane = fitPlane(coords);
                QList<int> outlierIndices = identifyOutliers(coords, plane, threshold_factor);

                if (outlierIndices.isEmpty() || coords.rows() - outlierIndices.size() < 4) {
                    break;
                }

                qDebug() << "Found" << outlierIndices.size() << "outliers";

                // Usuń outliers z zachowaniem minimum punktów
                Eigen::MatrixXd newCoords(coords.rows() - outlierIndices.size(), 3);
                int newIdx = 0;
                for (int i = 0; i < coords.rows(); i++) {
                    if (!outlierIndices.contains(i)) {
                        newCoords.row(newIdx++) = coords.row(i);
                    }
                }
                coords = newCoords;

                iteration++;
            } catch (const std::exception& e) {
                qDebug() << "Error in iteration:" << e.what();
                break;
            }
        }

        // Calculate final residuals
        residuals.resize(coords.rows());
        for (int i = 0; i < coords.rows(); i++) {
            residuals(i) = std::abs(plane.a * coords(i,0) + plane.b * coords(i,1) +
                                    plane.c * coords(i,2) + plane.d);
        }

        FitResult result;
        result.plane = plane;
        result.inlierCoords = coords;
        result.residuals = residuals;
        result.processedData = processed;
        return result;

    } catch (const std::exception& e) {
        qDebug() << "Error calculating coordinates:" << e.what();
        throw;
    }
}

void Draw::updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData) {
    try {
        std::vector<MultiPlaneDetector::Plane> planes = MultiPlaneDetector::detectMultiplePlanes(sensorData);
        QString description = MultiPlaneDetector::generatePlanesDescription(planes);

        if (planes.empty()) {
            ui->SurfaceLabel->setText("Nie wykryto żadnych płaszczyzn");
            ui->AngleLabel->setText("");
        } else {
            ui->SurfaceLabel->setText(QString("Wykryto %1 płaszczyzn:").arg(planes.size()));
            ui->AngleLabel->setText(description);
        }
    } catch (const std::exception& e) {
        qDebug() << "Error in updateLabels:" << e.what();
        ui->SurfaceLabel->setText("Błąd przetwarzania danych");
        ui->AngleLabel->setText("");
    }
}

QList<QList<int>> Draw::rotate90Right(const QList<QList<int>> &original) {
    qDebug() << "Rotating data, original size:" << original.size();

    QList<QList<int>> rotated;
    if (original.isEmpty()) {
        qDebug() << "Empty input data in rotate90Right";
        return rotated;
    }

    try {
        rotated.resize(8);
        for (int i = 0; i < 8; i++) {
            rotated[i].resize(8);
            for (int j = 0; j < 8; j++) {
                if (i < original.size() && (7-j) < original[i].size()) {
                    rotated[i][j] = original[7-j][i];
                } else {
                    qDebug() << "Invalid access at" << i << j;
                    return QList<QList<int>>();
                }
            }
        }
    }
    catch (const std::exception& e) {
        qDebug() << "Error in rotate90Right:" << e.what();
        return QList<QList<int>>();
    }

    return rotated;
}

void Draw::updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData) {


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

    // Bezpieczne obracanie danych
    QList<QList<int>> rotated;
    try {
        rotated = rotate90Right(sensorData);
    } catch (const std::exception& e) {
        qDebug() << "Błąd podczas obracania danych:" << e.what();
        return;
    }

    scene->clear();

    scene->clear();
    QRectF viewRect = ui->graphicsView->viewport()->rect();
    float increasedWidth = viewRect.width() * 1.10f;
    float increasedHeight = viewRect.height() * 1.10f;
    float squareSizeX = increasedWidth / 8.0f;
    float squareSizeY = increasedHeight / 8.0f;

    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            int value = rotated[i][j];
            QColor color;
            if (value <= 400) {
                int red = qMin(255, value * 255 / 400);
                color = QColor(red, 0, 0);
            } else {
                int green = qMin(255, (value - 400) * 255 / 600);
                int red = 255 - green;
                color = QColor(red, green, 0);
            }

            scene->addRect(j * squareSizeX, i * squareSizeY, squareSizeX, squareSizeY, QPen(Qt::black), QBrush(color));
            int fontSize = static_cast<int>(squareSizeX / 4);
            QGraphicsTextItem *text = scene->addText(QString::number(value), QFont("Arial", fontSize, QFont::Bold));
            text->setDefaultTextColor(Qt::white);
            text->setPos(j * squareSizeX + squareSizeX / 4.0f, i * squareSizeY + squareSizeY / 4.0f);
        }
    }

    ui->graphicsView->setSceneRect(0, 0, increasedWidth, increasedHeight);
    // Wywołaj updateLabels na przetworzonych danych
    updateLabels(ui, rotated);
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
