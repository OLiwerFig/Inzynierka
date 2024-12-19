#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include <QSerialPort>
#include <QGraphicsScene>
#include "ui_mainwindow.h"

class MainWindow;

class serialport : public QObject
{
    Q_OBJECT

public:
    explicit serialport(QObject *parent = nullptr);
    void connectSerialPort(const QString &portName);
    void refreshConnection(Ui::MainWindow *ui);
    void sendTargetCoordinates(Ui::MainWindow *ui);
    void handleSerialData(MainWindow *mainWindow, Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QByteArray> &values);
    void handlePortStatusChanged(Ui::MainWindow *ui, bool connected, const QString &portName);
    void populateAvailablePorts(Ui::MainWindow *ui);
    void sendFlag(Ui::MainWindow *ui);
    void sendMovementCommand(char command, int leftSpeed = -1, int rightSpeed = -1);

    QMap<char, QList<QList<int>>> sensorLastData;
    char currentSensor = '\0';

    // Zmienne dla przechowywania prędkości
    int currentSpeedLeft = 400;
    int currentSpeedRight = 400;

    // Funkcje ustawiania prędkości
    void setSpeed(int speed) {
        currentSpeedLeft = speed;
        currentSpeedRight = speed;
    }

    void setSpeeds(int leftSpeed, int rightSpeed) {
        currentSpeedLeft = leftSpeed;
        currentSpeedRight = rightSpeed;
    }

<<<<<<< Updated upstream
=======
    int getActiveSensorsCount() const {
        int count = 0;
        for (const auto& sensor : {'A', 'B', 'C'}) {
            if (!sensorLastData[sensor].isEmpty()) {
                count++;
            }
        }
        return count;
    }

>>>>>>> Stashed changes
signals:
    void serialDataReceived(const QList<QByteArray> &data);
    void portStatusChanged(bool connected, const QString &portName);

private:
    QSerialPort *serialPort;
};

#endif // SERIALPORT_H
