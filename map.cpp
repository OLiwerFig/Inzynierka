#include "map.h"
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QObject>
#include <QTimer>
#include <cmath>

Map::Map(QGraphicsView *view)
    : QObject(), scene(nullptr), robotItem(nullptr), robotTimer(nullptr) {
    if (!view) {
        qDebug() << "Błąd: przekazano nullptr jako QGraphicsView!";
        return;
    }

    scene = new QGraphicsScene(view);
    view->setScene(scene);
    qDebug() << "Scena utworzona i przypisana do widoku";

    robotItem = new QGraphicsEllipseItem(0, 0, 20, 20);
    if (!robotItem) {
        qDebug() << "Błąd: nie udało się utworzyć robotItem!";
        return;
    }
    robotItem->setBrush(Qt::red);
    robotItem->setPen(Qt::NoPen);
    qDebug() << "RobotItem utworzony poprawnie";
}

Map::~Map() {
    delete scene;
    if (robotTimer) {
        delete robotTimer;
    }
}
bool Map::checkCollision(const QPointF &position) {
    if (!scene || !robotItem) {
        return false;
    }

    // Tworzymy tymczasowy prostokąt reprezentujący obszar robota na nowej pozycji
    const qreal robotSize = 20;
    QRectF robotRect(
        position.x() - robotSize/2,
        position.y() - robotSize/2,
        robotSize,
        robotSize
        );

    qDebug() << "Sprawdzanie kolizji dla prostokąta:" << robotRect;

    // Najpierw sprawdź czy robot jest w dozwolonym obszarze (wewnątrz głównego kwadratu)
    if (position.x() < 130 || position.x() > 1170 ||
        position.y() < 130 || position.y() > 1170) {
        qDebug() << "Pozycja poza dozwolonym obszarem:" << position;
        return true;
    }

    // Sprawdź kolizje tylko z przeszkodami (pomijając linie brzegowe)
    foreach(QGraphicsItem* item, scene->items()) {
        if (item == robotItem) {
            continue;
        }

        // Sprawdź czy to jest przeszkoda (prostokąt wypełniony)
        QGraphicsRectItem* rectItem = qgraphicsitem_cast<QGraphicsRectItem*>(item);
        if (rectItem && rectItem->brush() != Qt::NoBrush) {
            QRectF obstacleRect = rectItem->sceneBoundingRect();
            // Dodaj margines bezpieczeństwa
            obstacleRect = obstacleRect.adjusted(-5, -5, 5, 5);

            if (obstacleRect.intersects(robotRect)) {
                qDebug() << "Wykryto kolizję z przeszkodą:" << obstacleRect;
                return true;
            }
        }
    }

    return false;
}

void Map::moveRobotUp() {
    qDebug() << "moveRobotUp called";
    if (!robotItem) {
        qDebug() << "robotItem is null";
        return;
    }
    QPointF currentPos = robotItem->scenePos();
    currentPos += QPointF(robotItem->rect().width()/2, robotItem->rect().height()/2); // Korekta na środek robota
    qDebug() << "Current position:" << currentPos;
    updateRobotPosition(currentPos.x(), currentPos.y() - ROBOT_STEP);
}

void Map::moveRobotDown() {
    qDebug() << "moveRobotDown called";
    if (!robotItem) return;
    QPointF currentPos = robotItem->scenePos();
    currentPos += QPointF(robotItem->rect().width()/2, robotItem->rect().height()/2);
    updateRobotPosition(currentPos.x(), currentPos.y() + ROBOT_STEP);
}

void Map::moveRobotLeft() {
    qDebug() << "moveRobotLeft called";
    if (!robotItem) return;
    QPointF currentPos = robotItem->scenePos();
    currentPos += QPointF(robotItem->rect().width()/2, robotItem->rect().height()/2);
    updateRobotPosition(currentPos.x() - ROBOT_STEP, currentPos.y());
}

void Map::moveRobotRight() {
    qDebug() << "moveRobotRight called";
    if (!robotItem) return;
    QPointF currentPos = robotItem->scenePos();
    currentPos += QPointF(robotItem->rect().width()/2, robotItem->rect().height()/2);
    updateRobotPosition(currentPos.x() + ROBOT_STEP, currentPos.y());
}


void Map::updateRobotPosition(qreal x, qreal y) {
    qDebug() << "updateRobotPosition called with:" << x << y;
    if (!robotItem || !scene) {
        qWarning() << "Robot lub scena nie zostały zainicjalizowane!";
        return;
    }

    try {
        // Oblicz nową pozycję z uwzględnieniem rozmiaru robota
        const qreal robotSize = 20;
        QPointF newPos(x, y);

        qDebug() << "Trying to move robot to:" << newPos;

        // Sprawdź kolizje przed ustawieniem nowej pozycji
        if (!checkCollision(newPos)) {
            // Brak kolizji - możemy ustawić nową pozycję
            robotItem->setPos(x - robotSize/2, y - robotSize/2); // Centrowanie robota
            qDebug() << "Robot przesunięty na pozycję:" << newPos;
        } else {
            qDebug() << "Nie można przesunąć robota - wykryto kolizję lub pozycja poza granicami";
        }

    } catch (const std::exception &e) {
        qCritical() << "Błąd podczas aktualizacji pozycji:" << e.what();
    }
}

void Map::initializeMap(const QString &filePath) {
    try {
        loadObstacles(filePath);
        QRectF bounds = scene->itemsBoundingRect();
        qDebug() << "Granice przed skalowaniem:" << bounds;

        if (robotItem) {
            scene->removeItem(robotItem);
        }

        scaleSceneToFitView();

        if (robotItem) {
            scene->addItem(robotItem);

            // Definiujemy bezpieczne pozycje początkowe wewnątrz głównego kwadratu
            const QPointF testPositions[] = {
                QPointF(150, 150),      // lewy górny róg
                QPointF(1150, 150),     // prawy górny róg
                QPointF(150, 1150),     // lewy dolny róg
                QPointF(1150, 1150),    // prawy dolny róg
                QPointF(650, 150),      // góra środek
                QPointF(650, 1150),     // dół środek
                QPointF(150, 650),      // lewo środek
                QPointF(1150, 650)      // prawo środek
            };

            bool positionFound = false;
            for (const QPointF &pos : testPositions) {
                if (!checkCollision(pos)) {
                    robotItem->setPos(pos.x() - 10, pos.y() - 10); // Centrowanie robota
                    positionFound = true;
                    qDebug() << "Znaleziono bezkolizyjną pozycję początkową:" << pos;
                    break;
                }
            }

            if (!positionFound) {
                qWarning() << "Nie znaleziono bezkolizyjnej pozycji początkowej!";
                // Fallback na bezpieczną pozycję
                robotItem->setPos(150 - 10, 150 - 10);
            }
        }

        if (!robotTimer) {
            robotTimer = new QTimer(this);
            connect(robotTimer, &QTimer::timeout, this, [this]() {
                if (robotItem) {
                    QPointF pos = robotItem->scenePos();
                    //qDebug() << "Bieżąca pozycja robota:" << pos;
                }
            });
            robotTimer->start(100);
        }

    } catch (const std::exception &e) {
        qCritical() << "Wyjątek podczas inicjalizacji mapy:" << e.what();
    } catch (...) {
        qCritical() << "Nieznany błąd podczas inicjalizacji mapy.";
    }
}


void Map::loadObstacles(const QString &filePath) {
    qDebug() << "Próba otwarcia pliku z przeszkodami:" << filePath;

    QFile file(filePath);
    if (!file.exists()) {
        qDebug() << "Plik" << filePath << "nie istnieje!";
        return;
    }

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Nie można otworzyć pliku z przeszkodami:" << filePath;
        return;
    }

    qDebug() << "Plik otwarty pomyślnie. Czyszczenie sceny.";
    scene->clear();
    qDebug() << "Liczba elementów przed wyczyszczeniem sceny: " << scene->items().size();

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        qDebug() << "Odczytano linię: " << line;

        if (line.startsWith("LINE")) {
            QStringList parts = line.split(' ');
            if (parts.size() == 6) {
                int x1 = parts[1].toInt();
                int y1 = parts[2].toInt();
                int x2 = parts[3].toInt();
                int y2 = parts[4].toInt();
                int thickness = parts[5].toInt();
                QPen pen(Qt::black);
                pen.setWidth(thickness);
                scene->addLine(x1, y1, x2, y2, pen);
                qDebug() << "Dodano linię od (" << x1 << "," << y1 << ") do (" << x2 << "," << y2 << ") z grubością " << thickness;
            }
        } else if (line.startsWith("RECT")) {
            QStringList parts = line.split(' ');
            if (parts.size() == 6 && parts[5] == "FILLED") {
                int x = parts[1].toInt();
                int y = parts[2].toInt();
                int width = parts[3].toInt();
                int height = parts[4].toInt();
                scene->addRect(x, y, width, height, QPen(Qt::black), QBrush(Qt::blue));
                qDebug() << "Dodano wypełniony prostokąt na pozycji (" << x << "," << y << ") o szerokości " << width << " i wysokości " << height;
            } else if (parts.size() == 5) {
                int x = parts[1].toInt();
                int y = parts[2].toInt();
                int width = parts[3].toInt();
                int height = parts[4].toInt();
                scene->addRect(x, y, width, height, QPen(Qt::black));
                qDebug() << "Dodano niewypełniony prostokąt na pozycji (" << x << "," << y << ") o szerokości " << width << " i wysokości " << height;
            }
        }
    }

    qDebug() << "Ładowanie przeszkód zakończone. Liczba elementów na scenie: " << scene->items().size();
}

void Map::scaleSceneToFitView() {
    if (!scene || scene->views().isEmpty()) {
        qWarning() << "Scena nie ma przypisanego widoku!";
        return;
    }

    QGraphicsView *view = scene->views().first();
    if (!view) {
        qWarning() << "Widok sceny jest nullptr!";
        return;
    }

    QRectF viewRect = view->viewport()->rect(); // Obszar widoku
    QRectF sceneRect = scene->itemsBoundingRect(); // Obszar elementów na scenie

    if (sceneRect.isNull() || viewRect.isNull()) {
        qWarning() << "Brak prawidłowych wymiarów sceny lub widoku!";
        return;
    }

    qreal scaleX = viewRect.width() / sceneRect.width();
    qreal scaleY = viewRect.height() / sceneRect.height();
    qreal scaleFactor = qMin(scaleX, scaleY);

    if (scaleFactor <= 0 || std::isnan(scaleFactor)) {
        qWarning() << "Nieprawidłowy współczynnik skalowania: " << scaleFactor;
        return;
    }

    QTransform transform;
    transform.scale(scaleFactor, scaleFactor);
    view->setTransform(transform);
    view->centerOn(sceneRect.center());

    qDebug() << "Scena przeskalowana do współczynnika:" << scaleFactor;
}
