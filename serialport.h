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
    void sendFlag(Ui::MainWindow *ui);
    void sendMovementCommand(char command);

    QMap<char, QList<QList<int>>> sensorLastData;
    char currentSensor = '\0';
    int currentSpeed = 400; // Domyślna prędkość
    void setSpeed(int speed) { currentSpeed = speed; }

    int getActiveSensorsCount() const {
        int count = 0;
        for (const auto& sensor : {'A', 'B', 'C'}) {
            if (!sensorLastData[sensor].isEmpty()) {
                count++;
            }
        }
        return count;
    }


signals:
    void activeSensorCountChanged(int count);
    void serialDataReceived(const QList<QByteArray> &data);
    void portStatusChanged(bool connected, const QString &portName);

private:
    QSerialPort *serialPort;
};

#endif // SERIALPORT_H
