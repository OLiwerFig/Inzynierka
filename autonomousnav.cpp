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
    if (rightSensor.isEmpty()) {
        return;
    }

    // Analiza danych z czujnika prawego (B)
    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    // Szukamy pierwszej płaszczyzny pionowej
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
        //qDebug() << "Not navigating or not currently turning.";
        return;
    }

    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    bool foundGoodAngle = false;
    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            double angle = plane.params.azimuth_angle;
            //qDebug() << "Turn sequence - right sensor angle:" << angle;

            if (angle >= -10 && angle <= 10) {
                foundGoodAngle = true;
                break;
            }
        }
    }

    bool pathClear = true;
    std::vector<MultiPlaneDetector::Plane> frontPlanes =
        MultiPlaneDetector::detectMultiplePlanes(frontSensor);

    for (const auto& plane : frontPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL &&
            std::abs(plane.params.d) < SAFE_DISTANCE) {
            pathClear = false;
            break;
        }
    }

    if (foundGoodAngle && pathClear) {
        qDebug() << "Turn complete - found good angle and path is clear.";
        isCurrentlyTurning = false;
        wallPID->reset();
        // Opcjonalnie: Wysłanie polecenia dojazdu do przodu po zakończeniu skrętu
        sendMovementCommand('F');
    } else {
        // Kontynuacja skrętu w lewo
        serialHandler->setSpeed(TURN_SPEED);
        sendMovementCommand('L');
    }
}

void AutonomousNav::followWallSequence(
    const QList<QList<int>>& leftSensor,
    const QList<QList<int>>& rightSensor) {

    if (!isNavigating) {
        //qDebug() << "Not navigating. Exiting followWallSequence.";
        return;
    }

    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    double wallAngle = -1000.0;
    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            wallAngle = plane.params.azimuth_angle;
            break;
        }
    }

    if (wallAngle != -1000.0) {
        qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        double correction = wallPID->calculate(0.0, wallAngle, deltaTime);
        correction = std::clamp(correction, -10.0, 10.0);

        int leftSpeed = BASE_SPEED;
        int rightSpeed = BASE_SPEED;

        if (correction > 0) {
            rightSpeed -= static_cast<int>(correction * 5);
        } else {
            leftSpeed += static_cast<int>(correction * 5);
        }

        leftSpeed = std::clamp(leftSpeed, 0, 400);
        rightSpeed = std::clamp(rightSpeed, 0, 400);

        serialHandler->setSpeed(leftSpeed);
        sendMovementCommand('F');

        //qDebug() << "Following wall - Angle:" << wallAngle
        //         << "Correction:" << correction
        //         << "Speeds L/R:" << leftSpeed << "/" << rightSpeed;
    } else {
        searchForWall();
    }
}

void AutonomousNav::handleFrontWall() {
    if (!isNavigating) {
        //qDebug() << "Not navigating. Cannot handle front wall.";
        return;
    }

    if (!isCurrentlyTurning) {
        qDebug() << "Starting turn sequence due to front wall.";
        isCurrentlyTurning = true;
        serialHandler->setSpeed(TURN_SPEED);
        sendMovementCommand('L');
        wallPID->reset();
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
