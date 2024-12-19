#include "autonomousnav.h"
#include "multiplanedetector.h"
#include <QDebug>
#include <algorithm>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QCoreApplication>

// Implementacja PIDController
PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), previousError(0), integral(0) {}

double PIDController::calculate(double setpoint, double currentValue, double deltaTime) {
    double error = setpoint - currentValue;
    integral += error * deltaTime;
    double derivative = (error - previousError) / deltaTime;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
    previousError = 0;
    integral = 0;
}

// Implementacja AutonomousNav
AutonomousNav::AutonomousNav(serialport* serial, QLabel* label, QObject *parent)
    : QObject(parent)
    , serialHandler(serial)
    , wallPID(new PIDController(0.8, 0.1, 0.3))
    , directionLabel(label)
    , isNavigating(false)
    , isCurrentlyTurning(false)
    , lastUpdateTime(QDateTime::currentMSecsSinceEpoch())
    , lastMovementCommand('S')
    , decelerationTimer(new QTimer(this))
    , currentSpeed(BASE_SPEED)
    , decelerationStep(50)
    , minimumSpeed(0)
    , lastAzimuthB(0.0)
    , logsDirectory("navigation_logs")  // Domyślny katalog na logi
{
    connect(serialHandler, &serialport::serialDataReceived,
            this, &AutonomousNav::onNewSensorData);

    if (directionLabel) {
        directionLabel->clear();
        lastCommands.clear();
    }

    QDir dir;
    if (!dir.exists(logsDirectory)) {
        dir.mkpath(logsDirectory);
    }

    // Konfiguracja timera dekrementacji prędkości
    connect(decelerationTimer, &QTimer::timeout, this, &AutonomousNav::performDeceleration);
}

void AutonomousNav::initializeLogging() {
    // Uzyskaj ścieżkę do katalogu aplikacji
    QString appPath = QCoreApplication::applicationDirPath();

    // Utwórz pełną ścieżkę do katalogu logów
    logsDirectory = appPath + "/navigation_logs";

    // Utwórz katalog jeśli nie istnieje
    QDir dir;
    if (!dir.exists(logsDirectory)) {
        dir.mkpath(logsDirectory);
    }

    // Generowanie nazwy pliku
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
    currentLogFileName = QString("%1/azimuth_log_%2.csv")
                             .arg(logsDirectory)
                             .arg(timestamp);

    logFile.setFileName(currentLogFileName);
    if (logFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        logStream.setDevice(&logFile);
        logStream << "Timestamp,AzimuthB(degrees)\n"; // Nagłówki
        qDebug() << "Pełna ścieżka do pliku logów:" << QDir::toNativeSeparators(currentLogFileName);
        qDebug() << "Katalog z logami znajduje się w:" << QDir::toNativeSeparators(logsDirectory);
    } else {
        qDebug() << "Błąd podczas otwierania pliku logów:" << currentLogFileName;
    }
}


void AutonomousNav::startNavigation() {
    isNavigating = true;
    isCurrentlyTurning = false;
    wallPID->reset();
    lastCommands.clear();
    lastUpdateTime = QDateTime::currentMSecsSinceEpoch();
    currentSpeed = BASE_SPEED;
    serialHandler->setSpeed(currentSpeed);
    sendMovementCommand('F');

    // Inicjalizacja logowania
    initializeLogging();
    lastAzimuthB = 0.0;

    qDebug() << "Autonomous navigation started";
}


void AutonomousNav::adjustAlignment() {
    auto rightSensor = serialHandler->sensorLastData['B'];
    if (rightSensor.isEmpty()) {
        qDebug() << "Brak danych z prawego czujnika. Nie mogę dostosować pozycji.";
        return;
    }

    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    double targetAzimuthMin = -10.0;
    double targetAzimuthMax = 10.0;
    bool needsAdjustment = false;
    double azimuth = 0.0;

    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            azimuth = plane.params.azimuth_angle;
            qDebug() << "Dostosowuję pozycję: Azimuth prawa = " << azimuth;
            if (azimuth < targetAzimuthMin) {
                needsAdjustment = true;
                sendMovementCommand('R'); // Skręt w prawo
                qDebug() << "Azimuth za niski. Skręcam w prawo.";
                break;
            } else if (azimuth > targetAzimuthMax) {
                needsAdjustment = true;
                sendMovementCommand('L'); // Skręt w lewo
                qDebug() << "Azimuth za wysoki. Skręcam w lewo.";
                break;
            }
        }
    }

    if (!needsAdjustment) {
        qDebug() << "Alignment within target azimuth range.";
        // Możesz zdecydować, co zrobić, gdy alignment jest poprawny
    }
}


void AutonomousNav::stopNavigation() {
    if (!isNavigating) {
        qDebug() << "Próba zatrzymania nawigacji, ale nie jest aktywna.";
        return;
    }

    isNavigating = false;
    isCurrentlyTurning = false;
    decelerationTimer->start(100);
    serialHandler->setSpeed(0);
    sendMovementCommand('S');
    wallPID->reset();
    updateDirectionLabel('S');

    // Zamknięcie pliku logów
    if (logFile.isOpen()) {
        logStream.flush();
        logFile.close();
        qDebug() << "Logowanie zatrzymane i plik zamknięty:" << currentLogFileName;
    }

    // Opcjonalne wysłanie dodatkowych sygnałów stop
    for(int i = 0; i < 5; ++i) {
        QTimer::singleShot(i * 100, this, [this]() {
            sendMovementCommand('S');
        });
    }
}



void AutonomousNav::sendMovementCommand(char command) {
    if (command != lastMovementCommand) {
        serialHandler->sendMovementCommand(command);
        lastMovementCommand = command;
        //qDebug() << "Sent movement command:" << command;
        updateDirectionLabel(command);
    } else {
        //qDebug() << "Movement command '" << command << "' already sent. Skipping.";
    }
}

void AutonomousNav::onNewSensorData() {
    if (!isNavigating) {
        return;
    }

    auto rightSensor = serialHandler->sensorLastData['B'];
    auto frontSensor = serialHandler->sensorLastData['C']; // Dodanie frontSensor

    if (rightSensor.isEmpty() || frontSensor.isEmpty()) {
        qDebug() << "Brak pełnych danych z czujników.";
        return;
    }

    // Logowanie surowych danych z czujników
    qDebug() << "Surowe dane z czujnika prawego (B):" << rightSensor;
    qDebug() << "Surowe dane z czujnika przedniego (C):" << frontSensor;

    // Analiza danych z czujnika prawego (B)
    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            double currentAzimuthB = plane.params.azimuth_angle;

            // Zapisujemy kąt tylko jeśli zmienił się znacząco
            const double AZIMUTH_THRESHOLD = 1.0; // Próg zmiany w stopniach
            if (std::abs(currentAzimuthB - lastAzimuthB) > AZIMUTH_THRESHOLD) {
                lastAzimuthB = currentAzimuthB;
                qint64 timestamp = QDateTime::currentMSecsSinceEpoch();

                if (logFile.isOpen()) {
                    logStream << timestamp << "," << currentAzimuthB << "\n";
                    logStream.flush(); // Zapewnienie natychmiastowego zapisu
                    qDebug() << "Zapisano nowy kąt azymutu. Timestamp:"
                             << timestamp << "AzimuthB:" << currentAzimuthB;
                }
            }
            break; // Wystarczy pierwsza znaleziona płaszczyzna pionowa
        }
    }



    // Analiza danych z czujnika przedniego (C)
    std::vector<MultiPlaneDetector::Plane> frontPlanes =
        MultiPlaneDetector::detectMultiplePlanes(frontSensor);



    // Kontynuacja standardowego przetwarzania nawigacji
    processNavigationStep();
}




void AutonomousNav::updateDirectionLabel(char command) {
    if (!directionLabel) return;

    QString commandStr = commandDescriptions.value(command, "Unknown");
    lastCommands.append(commandStr);

    while (lastCommands.size() > 4) {
        lastCommands.removeFirst();
    }

    QString labelText = "Last commands:\n";
    for (const QString& cmd : lastCommands) {
        labelText += cmd + "\n";
    }

    directionLabel->setText(labelText);
}

void AutonomousNav::processNavigationStep() {
    //qDebug() << "Processing navigation step. isNavigating:" << isNavigating;
    if (!isNavigating) {
        serialHandler->setSpeed(0);
        sendMovementCommand('S');
        updateDirectionLabel('S');
        //qDebug() << "Navigation is not active. Sent stop command.";
        return;
    }

    auto leftSensor = serialHandler->sensorLastData['A'];
    auto rightSensor = serialHandler->sensorLastData['B'];
    auto frontSensor = serialHandler->sensorLastData['C'];

    if (frontSensor.isEmpty() || leftSensor.isEmpty() || rightSensor.isEmpty()) {
        //qDebug() << "Incomplete sensor data. Skipping navigation step.";
        return;
    }

    if (checkFrontCollisionRisk(frontSensor)) {
        if (!isCurrentlyTurning) {
            //qDebug() << "Collision risk detected in front. Handling front wall.";
            handleFrontWall();
        }
        return;
    }

    if (isCurrentlyTurning) {
        //qDebug() << "Currently turning. Handling turning sequence.";
        handleTurningSequence(leftSensor, rightSensor, frontSensor);
    } else {
        //qDebug() << "Following wall sequence.";
        followWallSequence(leftSensor, rightSensor);
    }
}


bool AutonomousNav::checkFrontCollisionRisk(const QList<QList<int>>& frontSensor) {
    //qDebug() << "Checking front collision risk.";
    // Detect multiple planes in frontSensor
    std::vector<MultiPlaneDetector::Plane> frontPlanes =
        MultiPlaneDetector::detectMultiplePlanes(frontSensor);

    for (const auto& plane : frontPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            if (std::abs(plane.params.d) < FRONT_WALL_DISTANCE) {
                //qDebug() << "Front vertical wall detected with d:" << plane.params.d;
                return true;
            }
        }
    }

    return false;
}

void AutonomousNav::handleTurningSequence(
    const QList<QList<int>>& leftSensor,
    const QList<QList<int>>& rightSensor,
    const QList<QList<int>>& frontSensor) {

    if (!isNavigating || !isCurrentlyTurning) {
        qDebug() << "Nie nawiguję lub nie jestem w trakcie skrętu.";
        return;
    }

    // Analiza płaszczyzn z czujnika przedniego
    bool frontAzimuthInRange = false;
    bool frontDInRange = false;
    std::vector<MultiPlaneDetector::Plane> frontPlanes =
        MultiPlaneDetector::detectMultiplePlanes(frontSensor);

    qDebug() << "Wykryte płaszczyzny przednie: " << frontPlanes.size();

    for (const auto& plane : frontPlanes) {
        QString typeStr = (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) ? "VERTICAL" :
                              (plane.type == MultiPlaneDetector::PlaneType::HORIZONTAL) ? "HORIZONTAL" : "OTHER";
        qDebug() << "Detected plane type:" << typeStr;

        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            double angle = plane.params.azimuth_angle;
            double d = std::abs(plane.params.d);
            qDebug() << "Sprawdzam płaszczyznę przednią podczas skrętu: Azimuth =" << angle << ", d =" << d;

            // Warunek zakończenia skrętu
            if (angle >= -15.0 && angle <= 15.0 && d < 150.0) {
                frontAzimuthInRange = true;
                frontDInRange = true;
                qDebug() << "Warunki stopu spełnione: Azimuth =" << angle << ", d =" << d;
                break;
            }
        }
    }

    // Dodatkowy warunek: sprawdzenie, czy płaszczyzna na prawej stronie ma azymut bliski -10 do 10
    bool rightAzimuthInRange = false;
    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            double angle = plane.params.azimuth_angle;
            if (angle >= -10.0 && angle <= 10.0) {
                rightAzimuthInRange = true;
                qDebug() << "Azimuth płaszczyzny prawej strony w zakresie: " << angle;
                break;
            }
        }
    }

    // Decyzja o zakończeniu skrętu
    if (frontAzimuthInRange || rightAzimuthInRange) {
        qDebug() << "Zakończono skręt. Warunki zostały spełnione.";
        isCurrentlyTurning = false;
        wallPID->reset();
        sendMovementCommand('F'); // Powrót do jazdy do przodu
    } else {
        qDebug() << "Kontynuuję skręt w lewo. Szukam dobrego ustawienia.";
        // Kontynuacja skrętu w lewo
        serialHandler->setSpeed(TURN_SPEED);
        sendMovementCommand('L');
    }
}


void AutonomousNav::followWallSequence(
    const QList<QList<int>>& leftSensor,
    const QList<QList<int>>& rightSensor) {

    if (!isNavigating) {
        qDebug() << "Nie nawiguję. Kończę śledzenie ściany.";
        return;
    }

    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    double wallAngle = -1000.0;
    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            wallAngle = plane.params.azimuth_angle;
            qDebug() << "Ściana prawa - Azimuth =" << wallAngle;
            break;
        }
    }

    if (wallAngle != -1000.0) {
        // Sprawdzenie, czy azymut jest w zakresie [-10°, +10°]
        if (wallAngle < -10.0 || wallAngle > 10.0) {
            qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            lastUpdateTime = currentTime;

            double correction = wallPID->calculate(0.0, wallAngle, deltaTime);
            correction = std::clamp(correction, -10.0, 10.0); // Ograniczenie korekcji

            int leftSpeed = BASE_SPEED;
            int rightSpeed = BASE_SPEED;

            if (correction > 0) {
                rightSpeed -= static_cast<int>(correction * 5);
                qDebug() << "Korekcja PID pozytywna. Skręcam w prawo.";
            } else {
                leftSpeed += static_cast<int>(-correction * 5);
                qDebug() << "Korekcja PID negatywna. Skręcam w lewo.";
            }

            leftSpeed = std::clamp(leftSpeed, 0, 400);
            rightSpeed = std::clamp(rightSpeed, 0, 400);

            serialHandler->setSpeed(leftSpeed);
            sendMovementCommand('F');

            qDebug() << "Śledzenie ściany - Azimuth:" << wallAngle
                     << "Korekcja:" << correction
                     << "Prędkości L/R:" << leftSpeed << "/" << rightSpeed;
        } else {
            qDebug() << "Kąt ściany w zakresie docelowym. Nie wymaga korekty.";
            // Możesz zdecydować, czy wysłać polecenie do jazdy prosto
            sendMovementCommand('F');
        }
    } else {
        qDebug() << "Nie wykryto ściany prawej. Szukam ściany.";
        searchForWall();
    }
}




void AutonomousNav::handleFrontWall() {
    if (!isNavigating) {
        qDebug() << "Nie nawiguję. Nie mogę obsłużyć frontowej ściany.";
        return;
    }

    if (!isCurrentlyTurning) {
        // Sprawdzenie warunków na czujniku przednim
        bool frontAzimuthInRange = false;
        bool frontDInRange = false;
        auto frontSensor = serialHandler->sensorLastData['C']; // Zakładam, że 'C' to czujnik przedni
        if (!frontSensor.isEmpty()) {
            std::vector<MultiPlaneDetector::Plane> frontPlanes =
                MultiPlaneDetector::detectMultiplePlanes(frontSensor);
            for (const auto& plane : frontPlanes) {
                if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
                    double angle = plane.params.azimuth_angle;
                    double d = std::abs(plane.params.d); // Upewniamy się, że d jest dodatnie
                    qDebug() << "Sprawdzam płaszczyznę przednią: Azimuth =" << angle << ", d =" << d;
                    if (angle >= -15.0 && angle <= 15.0 && d < 120.0) {
                        frontAzimuthInRange = true;
                        frontDInRange = true;
                        break;
                    }
                }
            }
        }

        // Decyzja o skręcie
        if (frontAzimuthInRange && frontDInRange) {
            qDebug() << "Warunki do skrętu spełnione. Rozpoczynam skręt w lewo.";
            isCurrentlyTurning = true;
            serialHandler->setSpeed(TURN_SPEED);
            sendMovementCommand('L'); // Skręt w lewo, można dostosować do potrzeb
            wallPID->reset();
        } else {
            qDebug() << "Warunki do skrętu niespełnione. Dostosowuję pozycję.";
            // Inicjalizacja korekty pozycji bez pełnego skrętu
            adjustAlignment();
        }
    }
}

void AutonomousNav::searchForWall() {
    if (!isNavigating) {
        //qDebug() << "Not navigating. Cannot search for wall.";
        return;
    }

    //qDebug() << "No wall detected, searching...";
    serialHandler->setSpeed(150);
    sendMovementCommand('R');
}

void AutonomousNav::performDeceleration() {
    if (currentSpeed > minimumSpeed) {
        currentSpeed -= decelerationStep;
        if (currentSpeed < minimumSpeed) {
            currentSpeed = minimumSpeed;
        }

        // Aktualizacja prędkości na serialHandler
        serialHandler->setSpeed(currentSpeed);
        //qDebug() << "Decelerating... Current speed:" << currentSpeed;

        // Opcjonalnie, możesz aktualizować etykietę kierunku
        updateDirectionLabel('F'); // Zakładając, że 'F' oznacza jazdę do przodu

    } else {
        // Prędkość osiągnęła zero, zatrzymujemy dekrementację
        decelerationTimer->stop();
        sendMovementCommand('S');
        //qDebug() << "Robot zatrzymany płynnie.";
    }
}
