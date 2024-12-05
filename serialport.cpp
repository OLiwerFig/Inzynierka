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

void serialport::sendFlag(Ui::MainWindow *ui) {
    QString selected = ui->FlagiComboBox->currentText();
    QByteArray data;

    if (selected == "Czujnik 1") {
        data = "A\n";
    } else if (selected == "Czujnik 2") {
        data = "B\n";
    } else if (selected == "Czujnik 3") {
        data = "C\n";
    }

    serialPort->write(data);
    qDebug() << "Wysłano flagę:" << data;
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
    for (const QByteArray &line : values) {
        QString trimmedLine = QString::fromUtf8(line).trimmed();

        if (trimmedLine == "A" || trimmedLine == "B" || trimmedLine == "C") {
            // Otrzymaliśmy informacje o tym, z którego czujnika będą następne dane
            currentSensor = trimmedLine.at(0).toLatin1();
        } else {
            // Ta linia powinna zawierać 64 wartości pomiarowe
            QStringList dataList = trimmedLine.split(' ', Qt::SkipEmptyParts);
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

                // Zapisz otrzymane dane w mapie
                sensorLastData[currentSensor] = sensorData;

                // Sprawdź, który czujnik jest obecnie wybrany w QComboBox
                QString selected = ui->FlagiComboBox->currentText();
                char selectedChar = '\0';
                if (selected == "Czujnik 1") selectedChar = 'A';
                else if (selected == "Czujnik 2") selectedChar = 'B';
                else if (selected == "Czujnik 3") selectedChar = 'C';

                // Jeśli aktualnie wybrany czujnik to ten, z którego właśnie przyszły dane - wyświetl je
                if (selectedChar == currentSensor) {
                    QMetaObject::invokeMethod(mainWindow, [ui, scene, sensorData]() {
                            Draw::updateSensorData(ui, scene, sensorData);
                        }, Qt::QueuedConnection);
                }

            } else {
                qDebug() << "Oczekiwano 64 wartości, odebrano: " << dataList.size();
            }
        }
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
