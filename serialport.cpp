#include "serialport.h"
#include "mainwindow.h"
#include "draw.h"
#include "ui_mainwindow.h"
#include <QMetaObject>
#include <QSerialPortInfo>
#include <QDataStream>

serialport::serialport(QObject *parent) : QObject(parent)
{
    serialPort = new QSerialPort(this);

    connect(serialPort, &QSerialPort::readyRead, this, [this]() {
        QList<QByteArray> data;
        while (serialPort->canReadLine()) {
            data.append(serialPort->readLine().trimmed());
        }
        emit serialDataReceived(data);

        // Debugowanie odebranych danych
        qDebug() << "Odebrano dane:" << data;
    });

    connect(serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::ResourceError) {
            serialPort->close();
            emit portStatusChanged(false, "");
        }
    });
}

void serialport::connectSerialPort(const QString &portName)
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }

    serialPort->setPortName(portName);
    serialPort->setBaudRate(QSerialPort::Baud115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (serialPort->open(QIODevice::ReadWrite)) {
        emit portStatusChanged(true, portName);
        qDebug() << "Połączono z portem:" << portName;
    } else {
        emit portStatusChanged(false, portName);
        qDebug() << "Nie udało się połączyć z portem:" << portName;
    }
}

void serialport::refreshConnection(Ui::MainWindow *ui) {
    connectSerialPort(ui->comboBoxSerialPorts->currentText());
}

void serialport::sendTargetCoordinates(Ui::MainWindow *ui) {
    bool okX, okY;
    float targetX = ui->lineEditTargetX->text().toFloat(&okX);
    float targetY = ui->lineEditTargetY->text().toFloat(&okY);

    if (okX && okY) {
        QByteArray data;
        QDataStream stream(&data, QIODevice::WriteOnly);
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
        stream.setByteOrder(QDataStream::LittleEndian);

        stream << targetX << targetY;
        serialPort->write(data);
        qDebug() << "Wysłano współrzędne: " << targetX << ", " << targetY;
    } else {
        qDebug() << "Nieprawidłowe współrzędne celu";
    }
}

void serialport::populateAvailablePorts(Ui::MainWindow *ui) {
    const auto serialPortInfos = QSerialPortInfo::availablePorts();
    ui->comboBoxSerialPorts->clear();
    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        ui->comboBoxSerialPorts->addItem(serialPortInfo.portName());
    }
}

void serialport::handleSerialData(MainWindow *mainWindow, Ui::MainWindow *ui, QGraphicsScene *scene, const QList<QByteArray> &values)
{
    // Sprawdzenie, czy mamy pełne dane
    QStringList dataList = QString(values.join(' ')).split(' ');

    if (dataList.size() == 64) {
        QList<QList<int>> sensorData;
        for (int i = 0; i < 8; ++i) {
            QList<int> row;
            for (int j = 0; j < 8; ++j) {
                int value = dataList[i * 8 + j].toInt();
                row.append(value);
            }
            sensorData.append(row);
        }

        // Debugowanie przetworzonych danych
        qDebug() << "Przetworzone dane z czujnika:";
        for (const auto &row : sensorData) {
            qDebug() << row;
        }

        // Aktualizacja danych na scenie graficznej
        QMetaObject::invokeMethod(mainWindow, [ui, scene, sensorData]() {
                Draw::updateSensorData(ui, scene, sensorData);
            }, Qt::QueuedConnection);
    } else {
        qDebug() << "Oczekiwano 64 wartości, odebrano: " << dataList.size();
    }
}


void serialport::handlePortStatusChanged(Ui::MainWindow *ui, bool connected, const QString &portName) {
    if (connected) {
        ui->ledIndicator->setStyleSheet("background-color: green; border-radius: 10px;");
        ui->statusLabel->setText(tr("Connected to ") + portName);
    } else {
        ui->ledIndicator->setStyleSheet("background-color: red; border-radius: 10px;");
        ui->statusLabel->setText(tr("Disconnected"));
    }
}
