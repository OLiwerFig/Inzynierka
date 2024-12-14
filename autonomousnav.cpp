#include "autonomousnav.h"
#include "multiplanedetector.h"
#include <QDebug>
#include <algorithm>

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
    , wallPID(new PIDController(0.8, 0.1, 0.3)) // Ustawienie domyślnych wartości PID
    , directionLabel(label)
    , isNavigating(false)
    , isCurrentlyTurning(false)
    , lastUpdateTime(QDateTime::currentMSecsSinceEpoch())
    , lastMovementCommand('S') // Inicjalizacja do 'S' (Stop)
{
    connect(serialHandler, &serialport::serialDataReceived,
            this, &AutonomousNav::onNewSensorData);

    if (directionLabel) {
        directionLabel->clear();
        lastCommands.clear();
    }
}

void AutonomousNav::startNavigation() {
    isNavigating = true;
    isCurrentlyTurning = false;
    wallPID->reset();
    lastCommands.clear();
    lastUpdateTime = QDateTime::currentMSecsSinceEpoch();
    qDebug() << "Autonomous navigation started";
}

void AutonomousNav::stopNavigation() {
    if (!isNavigating) {
        qDebug() << "Attempted to stop navigation, but it was not active.";
        return;
    }

    isNavigating = false;
    isCurrentlyTurning = false;
    serialHandler->setSpeed(0); // Ustawienie prędkości na 0
    sendMovementCommand('S');
    wallPID->reset();
    updateDirectionLabel('S');
    qDebug() << "Autonomous navigation stopped";

    // Opcjonalnie: Wysłanie kilku sygnałów 'S' aby upewnić się, że robot zatrzyma się
    for(int i = 0; i < 5; ++i) {
        QTimer::singleShot(i * 100, this, [this]() {
            sendMovementCommand('S');
            qDebug() << "Sent stop command 'S'";
        });
    }
}

void AutonomousNav::sendMovementCommand(char command) {
    if (command != lastMovementCommand) {
        serialHandler->sendMovementCommand(command);
        lastMovementCommand = command;
        qDebug() << "Sent movement command:" << command;
        updateDirectionLabel(command);
    }
}

void AutonomousNav::onNewSensorData() {
    if (!isNavigating) {
        qDebug() << "Received sensor data, but navigation is stopped.";
        return;
    }

    auto leftSensor = serialHandler->sensorLastData['A'];
    auto rightSensor = serialHandler->sensorLastData['B'];
    auto frontSensor = serialHandler->sensorLastData['C'];

    if (leftSensor.isEmpty() || rightSensor.isEmpty() || frontSensor.isEmpty()) {
        return;
    }

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
    if (!isNavigating) {
        serialHandler->setSpeed(0);
        sendMovementCommand('S');
        updateDirectionLabel('S');
        return;
    }

    auto leftSensor = serialHandler->sensorLastData['A'];
    auto rightSensor = serialHandler->sensorLastData['B'];
    auto frontSensor = serialHandler->sensorLastData['C'];

    if (checkCollisionRisk(frontSensor)) {
        if (!isCurrentlyTurning) {
            handleFrontWall();
        }
        return;
    }

    if (isCurrentlyTurning) {
        handleTurningSequence(leftSensor, rightSensor, frontSensor);
    } else {
        followWallSequence(leftSensor, rightSensor);
    }
}

bool AutonomousNav::checkCollisionRisk(const QList<QList<int>>& frontSensor) {
    for (int i = 3; i <= 5; i++) {
        for (int j = 3; j <= 5; j++) {
            if (frontSensor[i][j] < SAFE_DISTANCE) {
                qDebug() << "Front obstacle detected at" << frontSensor[i][j] << "mm";
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

    if (!isNavigating || !isCurrentlyTurning) return;

    std::vector<MultiPlaneDetector::Plane> rightPlanes =
        MultiPlaneDetector::detectMultiplePlanes(rightSensor);

    bool foundGoodAngle = false;
    for (const auto& plane : rightPlanes) {
        if (plane.type == MultiPlaneDetector::PlaneType::VERTICAL) {
            double angle = plane.params.azimuth_angle;
            qDebug() << "Turn sequence - right sensor angle:" << angle;

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
        qDebug() << "Turn complete - found good angle and path is clear";
        isCurrentlyTurning = false;
        wallPID->reset();
    } else {
        // Zamiast bezpośrednio wysyłać 'L', użyj metody sendMovementCommand
        serialHandler->setSpeed(TURN_SPEED);
        sendMovementCommand('L');
    }
}

void AutonomousNav::followWallSequence(
    const QList<QList<int>>& leftSensor,
    const QList<QList<int>>& rightSensor) {

    if (!isNavigating) return;

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
        correction = std::clamp(correction, -30.0, 30.0);

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
        // Użyj metody sendMovementCommand zamiast bezpośredniego wysyłania 'F'
        sendMovementCommand('F');

        qDebug() << "Following wall - Angle:" << wallAngle
                 << "Correction:" << correction
                 << "Speeds L/R:" << leftSpeed << "/" << rightSpeed;
    } else {
        searchForWall();
    }
}

void AutonomousNav::handleFrontWall() {
    if (!isNavigating) return;

    if (!isCurrentlyTurning) {
        qDebug() << "Starting turn sequence";
        isCurrentlyTurning = true;
        serialHandler->setSpeed(TURN_SPEED);
        sendMovementCommand('L');
        wallPID->reset();
    }
}

void AutonomousNav::searchForWall() {
    if (!isNavigating) return;

    qDebug() << "No wall detected, searching...";
    serialHandler->setSpeed(150);
    sendMovementCommand('R');
}
