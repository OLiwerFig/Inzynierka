#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QTranslator>
#include "serialport.h"
#include "setview.h"
#include "map.h"
#include "autonomousnav.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void updatePortStatus(bool connected);

private:
    Ui::MainWindow *ui;
    serialport *serialPortHandler;
    SetView *setViewHandler;
    QGraphicsScene *scene;
    QGraphicsPixmapItem *gridPixmapItem;
    QList<QPointF> pathPoints;
    QList<QGraphicsEllipseItem *> pathItems;
    QTranslator *translator;
    void onFlagChanged();
    void resizeEvent(QResizeEvent *event) override;
    void initializeMap();
    Map *map;
    void onSpeedSliderChanged(int value);
       void toggleAutoNavigation();
       void updateSensorCount(int count);

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void timerEvent(QTimerEvent *event) override;
        bool isAutoNavActive;
    AutonomousNav *autoNav;


};

#endif // MAINWINDOW_H
