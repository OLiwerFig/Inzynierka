#include "draw.h"
#include <QString>
#include <cmath>
#include <Eigen/Dense>
#include <QDebug>
#include <QGraphicsTextItem>
#include <QPen>
#include <QBrush>

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

QList<QList<int>> Draw::rotate90Right(const QList<QList<int>> &original) {
    QList<QList<int>> rotated;
    rotated.resize(8);
    for (int i = 0; i < 8; i++) {
        rotated[i].resize(8);
    }

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            rotated[i][j] = original[7 - j][i];
        }
    }

    return rotated;
}

static double neighborMean(const QList<QList<int>> &data, int i, int j) {
    double sum = 0.0;
    int count = 0;
    for (int di = -1; di <= 1; di++) {
        for (int dj = -1; dj <= 1; dj++) {
            if (di == 0 && dj == 0) continue;
            int ni = i + di;
            int nj = j + dj;
            if (ni >= 0 && ni < 8 && nj >= 0 && nj < 8) {
                sum += data[ni][nj];
                count++;
            }
        }
    }
    if (count > 0) return sum / count;
    return data[i][j];
}

QList<QList<int>> Draw::preprocessSensorData(const QList<QList<int>> &sensorData, int max_outliers) {
    QList<QList<int>> data = sensorData;
    QList<QPair<int,int>> outlierIndices;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
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

Eigen::MatrixXd Draw::calculateCoordinates(const QList<QList<int>> &sensorData) {
    Eigen::MatrixXd coords(64,3);
    int index = 0;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            double d_value = sensorData[i][j];
            // konwersja na radiany
            double elev = rotatedAnglesX[i][j]*PI/180.0;
            double azim = rotatedAnglesZ[i][j]*PI/180.0;

            double x_val = d_value * std::cos(elev) * std::sin(azim);
            double y_val = d_value * std::sin(elev);
            double z_val = d_value * std::cos(elev) * std::cos(azim);

            coords(index,0)=x_val;
            coords(index,1)=y_val;
            coords(index,2)=z_val;
            index++;
        }
    }

    return coords;
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
    for (int i=1;i<3;i++){
        if (eigvals(i)<min_val) {
            min_val = eigvals(i);
            idx_min = i;
        }
    }

    Eigen::Vector3d normal_vector = eigvecs.col(idx_min);
    double a = normal_vector(0);
    double b = normal_vector(1);
    double c = normal_vector(2);

    double d = - (a * S_x1 + b * S_y1 + c * S_z1) / Np;

    double norm = sqrt(a*a + b*b + c*c);
    if (norm > 1e-12) {
        a /= norm; b/= norm; c/= norm; d/= norm;
    } else {
        a=0; b=0; c=1; d=0;
    }

    PlaneParams p{a,b,c,d};
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
    // Preprocessing
    QList<QList<int>> processed = preprocessSensorData(sensorData,3);

    Eigen::MatrixXd coords_all = calculateCoordinates(processed);
    Eigen::MatrixXd coords = coords_all;

    int iteration=0;
    int total_outliers=0;

    PlaneParams plane;
    Eigen::VectorXd residuals;

    while (iteration < max_iterations) {
        plane = fitPlane(coords);

        QList<int> outlierIndices = identifyOutliers(coords, plane, threshold_factor);
        if (outlierIndices.isEmpty()) {
            qDebug()<<"No outliers detected in iteration"<<iteration;
            break;
        } else {
            int num_new_outliers = outlierIndices.size();
            total_outliers += num_new_outliers;
            if (total_outliers > max_outliers) {
                int num_allowed = max_outliers - (total_outliers - num_new_outliers);
                if (num_allowed < 0) num_allowed=0;
                if (num_allowed > 0) {
                    qDebug()<<"Iteration"<<iteration<<": Detected"<<num_new_outliers<<"outlier(s). Only excluding"<<num_allowed<<"more outlier(s).";
                    QSet<int> outSet;
                    for (int q=0; q<num_allowed; q++) outSet.insert(outlierIndices[q]);

                    Eigen::MatrixXd newCoords(coords.rows()-num_allowed,3);
                    int idxNew=0;
                    for (int r=0;r<coords.rows();r++){
                        if (!outSet.contains(r)) {
                            newCoords.row(idxNew++)=coords.row(r);
                        }
                    }
                    coords = newCoords;
                } else {
                    qDebug()<<"No more outliers allowed to remove.";
                }
                break;
            } else {
                qDebug()<<"Iteration"<<iteration<<": Detected and removed"<<num_new_outliers<<"outlier(s).";
                QSet<int> outSet;
                for (int q=0;q<num_new_outliers;q++) outSet.insert(outlierIndices[q]);
                Eigen::MatrixXd newCoords(coords.rows()-num_new_outliers,3);
                int idxNew=0;
                for (int r=0;r<coords.rows();r++){
                    if (!outSet.contains(r)) {
                        newCoords.row(idxNew++)=coords.row(r);
                    }
                }
                coords = newCoords;
                iteration++;
            }
        }
    }

    residuals.resize(coords.rows());
    for (int i=0;i<coords.rows();i++) {
        double rx=coords(i,0), ry=coords(i,1), rz=coords(i,2);
        double dist = std::abs(plane.a*rx + plane.b*ry + plane.c*rz + plane.d);
        residuals(i)=dist;
    }

    FitResult result;
    result.plane=plane;
    result.inlierCoords=coords;
    result.residuals=residuals;
    result.processedData=processed;
    return result;
}

void Draw::updateLabels(Ui::MainWindow *ui, const QList<QList<int>> &sensorData) {
    // Wyznaczamy płaszczyznę i kąty
    FitResult res = iterativePlaneFitting(sensorData,5,20,2.5);
    double a = res.plane.a;
    double b = res.plane.b;
    double c = res.plane.c;
    double d = res.plane.d;

    QString planeText = QString("Współczynniki płaszczyzny: a=%1, b=%2, c=%3, d=%4")
                            .arg(a).arg(b).arg(c).arg(d);
    ui->SurfaceLabel->setText(planeText);

    // Kąty
    double angle_x = std::atan(std::abs(a)/std::sqrt(b*b+c*c))*180.0/PI;
    double angle_z = std::atan(std::abs(c)/std::sqrt(a*a+b*b))*180.0/PI;

    QString angleText = QString("Kąt do osi X: %1°, Kąt do osi Z: %2°")
                            .arg(angle_x)
                            .arg(angle_z);
    ui->AngleLabel->setText(angleText);
}

void Draw::updateSensorData(Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QList<int>> &sensorData) {
    // Obróć dane o 90 stopni w prawo
    QList<QList<int>> rotated = rotate90Right(sensorData);

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
