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
        //qDebug() << "Odebrano dane:" << data;
    });

    connect(serialPort, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (error == QSerialPort::ResourceError) {
            serialPort->close();
            emit portStatusChanged(false, "");
        }
    });
}

void serialport::sendMovementCommand(char command) {
    if (serialPort->isOpen()) {
        // Format: $CMD,SPEED#
        QByteArray data;
        data.append('$');  // Znacznik początku
        data.append(command);  // Komenda ruchu (F/B/L/R/S)
        data.append(',');
        data.append(QString::number(currentSpeed).toUtf8());  // Prędkość
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
            qDebug() << "Znaleziono dane PWM";
            QStringList parts = trimmedLine.split(',');
            qDebug() << "Części:" << parts;

            if (parts.size() >= 3) {
                // Usuń znaki $ i # z wartości
                QString leftStr = parts[1];
                QString rightStr = parts[2];
                rightStr.remove('#');

                bool okLeft, okRight;
                float leftPWM = leftStr.toFloat(&okLeft);
                float rightPWM = rightStr.toFloat(&okRight);

                if (okLeft && okRight) {
                    qDebug() << "Wartości PWM - Lewy:" << leftPWM << "Prawy:" << rightPWM;

                    // Aktualizacja tekstów
                    ui->speed_text_L->setText(QString("Left PWM: %1").arg(leftPWM));
                    ui->speed_text_R->setText(QString("Right PWM: %1").arg(rightPWM));

                    // Aktualizacja progress barów
                    ui->speedLProgressBar->setValue(static_cast<int>(leftPWM));
                    ui->speedRProgressBar->setValue(static_cast<int>(rightPWM));
                }
            }
            continue;
        }



        // Sprawdź czy to potwierdzenie komendy
        if (trimmedLine.startsWith("CMD_")) {
            qDebug() << "Otrzymano potwierdzenie:" << trimmedLine;
            continue;
        }

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
