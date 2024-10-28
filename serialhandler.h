#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include <QObject>
#include <QSerialPort>

class SerialHandler : public QObject {
    Q_OBJECT

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler();

signals:
    void dataReceived(float speed_L, float speed_R, float pwm_L, float pwm_R, float odom_x, float odom_y, float odom_theta);

private slots:
    void readData();

private:
    QSerialPort *serial;
};

#endif // SERIALHANDLER_H
