#ifndef MAP_H
#define MAP_H
#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsView>
#include <QString>
#include <QTimer>

class Map : public QObject {
    Q_OBJECT

public:
    explicit Map(QGraphicsView *view);
    ~Map();
    void initializeMap(const QString &filePath);
    void loadObstacles(const QString &filePath);
    void updateRobotPosition(qreal x, qreal y);
    void scaleSceneToFitView();

public slots:  // Dodajemy slots aby jasno określić, że są to sloty
    void moveRobotUp();
    void moveRobotDown();
    void moveRobotLeft();
    void moveRobotRight();

private:
    const qreal ROBOT_STEP = 10.0;
    bool checkCollision(const QPointF &position);  // Zostawiamy tylko jedną wersję
    QGraphicsScene *scene;
    QGraphicsEllipseItem *robotItem;
    QTimer *robotTimer;
    QPointF calculateRobotInitialPosition();
};
#endif // MAP_H
