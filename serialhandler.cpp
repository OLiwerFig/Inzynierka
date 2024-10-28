#include "serialhandler.h"
#include <QDebug>
#include <QRegularExpression>

SerialHandler::SerialHandler(QObject *parent) : QObject(parent), serial(new QSerialPort(this)) {
    // Konfiguracja portu szeregowego
    serial->setPortName("/dev/tty.usbmodem1303"); // Ustawienie nazwy portu
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (serial->open(QIODevice::ReadOnly)) {
        connect(serial, &QSerialPort::readyRead, this, &SerialHandler::readData);
        qDebug() << "Port szeregowy otwarty";
    } else {
        qDebug() << "Nie udało się otworzyć portu szeregowego";
    }
}

SerialHandler::~SerialHandler() {
    delete serial;
}

void SerialHandler::readData() {
    static QByteArray buffer;
    buffer.append(serial->readAll());

    // Log the raw data received
    qDebug() << "Raw Data: " << buffer.toHex();

    // Check for complete message (assuming messages end with a newline character)
    int endIdx = buffer.indexOf('\n');
    if (endIdx != -1) {
        QByteArray completeMessage = buffer.left(endIdx);
        buffer.remove(0, endIdx + 1);

        qDebug() << "Complete Message: " << completeMessage;

        // Convert data to string and split into individual values
        QString receivedData = QString::fromLatin1(completeMessage);
        QStringList dataList = receivedData.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts); // Split by whitespace

        // Check if there are enough values to parse
        if (dataList.size() == 7) {
            // Parse the first 7 values
            float speedL = dataList[0].toFloat();
            float speedR = dataList[1].toFloat();
            float pwmL = dataList[2].toFloat();
            float pwmR = dataList[3].toFloat();
            float odometryX = dataList[4].toFloat();
            float odometryY = dataList[5].toFloat();
            float odometryTheta = dataList[6].toFloat();

            // Emit the dataReceived signal
            emit dataReceived(speedL, speedR, pwmL, pwmR, odometryX, odometryY, odometryTheta);
        } else {
            // Log an error if the number of values is insufficient
            qDebug() << "Błąd: Nieprawidłowa liczba danych";
        }
    }
}
