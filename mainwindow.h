#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QTranslator>
#include "serialport.h"
#include "setview.h"

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
    void resizeEvent(QResizeEvent *event) override;
};

#endif // MAINWINDOW_H
