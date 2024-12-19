#include "serialport.h"
#include "mainwindow.h"
#include "multiplanedetector.h"
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
        //qDebug() << "Odebrano dane:" << data;
    });

    connect(serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::ResourceError) {
            serialPort->close();
            emit portStatusChanged(false, "");
        }
    });
}

void serialport::sendMovementCommand(char command, int leftSpeed, int rightSpeed) {
    if (serialPort->isOpen()) {
        // Jeśli nie podano konkretnych prędkości, użyj aktualnych wartości
        if (leftSpeed == -1) leftSpeed = currentSpeedLeft;
        if (rightSpeed == -1) rightSpeed = currentSpeedRight;

        // Format: $CMD,SPEEDL,SPEEDR#
        QByteArray data;
        data.append('$');                                    // Znacznik początku
        data.append(command);                               // Komenda ruchu (F/B/L/R/S)
        data.append(',');
        data.append(QString::number(leftSpeed).toUtf8());   // Prędkość lewego silnika
        data.append(',');
        data.append(QString::number(rightSpeed).toUtf8());  // Prędkość prawego silnika
        data.append('#');
        data.append('\n');

        serialPort->write(data);
        qDebug() << "Wysłano komendę:" << data;
    }
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
        qDebug() << "Otrzymano linię:" << trimmedLine;

        if (trimmedLine.startsWith("$PWM")) {
            // Obsługa PWM bez zmian
            QStringList parts = trimmedLine.split(',');
            if (parts.size() >= 3) {
                QString leftStr = parts[1];
                QString rightStr = parts[2];
                rightStr.remove('#');

                bool okLeft, okRight;
                float leftPWM = leftStr.toFloat(&okLeft);
                float rightPWM = rightStr.toFloat(&okRight);

                if (okLeft && okRight) {
                    ui->speed_text_L->setText(QString("Left PWM: %1").arg(leftPWM));
                    ui->speed_text_R->setText(QString("Right PWM: %1").arg(rightPWM));

                    ui->speedLProgressBar->setValue(static_cast<int>(leftPWM));
                    ui->speedRProgressBar->setValue(static_cast<int>(rightPWM));
                }
            }
            continue;
        }

        if (trimmedLine.startsWith("CMD_")) {
            qDebug() << "Otrzymano potwierdzenie:" << trimmedLine;
            continue;
        }

        if (trimmedLine == "A" || trimmedLine == "B" || trimmedLine == "C") {
            currentSensor = trimmedLine.at(0).toLatin1();
            qDebug() << "Zmieniono aktywny czujnik na:" << currentSensor;
        } else {
            // Przetwarzanie danych z czujnika (8x8)
            QStringList dataList = trimmedLine.split(' ', Qt::SkipEmptyParts);

            if (dataList.size() != 64) {
                qDebug() << "Błąd: Nieprawidłowa liczba wartości:" << dataList.size();
                continue;
            }

            QList<QList<int>> sensorData;
            bool conversionOk = true;

            try {
                for (int i = 0; i < 8 && conversionOk; ++i) {
                    QList<int> row;
                    for (int j = 0; j < 8 && conversionOk; ++j) {
                        int index = i * 8 + j;
                        bool ok;
                        int value = dataList[index].toInt(&ok);
                        if (!ok) {
                            qDebug() << "Błąd konwersji wartości:" << dataList[index];
                            conversionOk = false;
                            break;
                        }
                        row.append(value);
                    }
                    if (conversionOk) {
                        sensorData.append(row);
                    }
                }

                if (conversionOk && sensorData.size() == 8 && sensorData[0].size() == 8) {
                    sensorLastData[currentSensor] = sensorData;

                    // Sprawdzamy, czy aktywny czujnik jest tym wybranym w interfejsie
                    QString selected = ui->FlagiComboBox->currentText();
                    char selectedChar = '\0';
                    if (selected == "Czujnik 1") selectedChar = 'A';
                    else if (selected == "Czujnik 2") selectedChar = 'B';
                    else if (selected == "Czujnik 3") selectedChar = 'C';

                    if (selectedChar == currentSensor) {
                        // Obracamy dane zaraz po odebraniu
                        QList<QList<int>> rotatedData = Draw::rotate90Right(sensorData);

                        // Wykryj płaszczyzny na obróconych danych
                        std::vector<MultiPlaneDetector::Plane> planes = MultiPlaneDetector::detectMultiplePlanes(rotatedData);

                        // Wywołaj updateSensorData z obróconymi danymi
                        QMetaObject::invokeMethod(mainWindow, [ui, scene, rotatedData, planes]() {
                                Draw::updateSensorData(ui, scene, rotatedData, planes);
                            }, Qt::QueuedConnection);
                    }
                } else {
                    qDebug() << "Błąd: Nieprawidłowy format danych czujnika";
                }
            } catch (const std::exception& e) {
                qDebug() << "Błąd podczas przetwarzania danych:" << e.what();
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
