#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include <QSerialPort>
#include <QGraphicsScene>
#include "ui_mainwindow.h"

// Forward declare MainWindow
class MainWindow;

class serialport : public QObject
{
    Q_OBJECT

public:
    explicit serialport(QObject *parent = nullptr);
    void connectSerialPort(const QString &portName);
    void refreshConnection(Ui::MainWindow *ui);
    void sendTargetCoordinates(Ui::MainWindow *ui);
    void handleSerialData(MainWindow *mainWindow, Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QByteArray> &values); // Ensure this matches the .cpp definition
    void handlePortStatusChanged(Ui::MainWindow *ui, bool connected, const QString &portName);
    void populateAvailablePorts(Ui::MainWindow *ui);

signals:
    void serialDataReceived(const QList<QByteArray> &data);
    void portStatusChanged(bool connected, const QString &portName);

private:
    QSerialPort *serialPort;
};

#endif // SERIALPORT_H
