#include "autonomousnav.h"
#include <QDebug>
#include <algorithm>

AutonomousNav::AutonomousNav(serialport* serial, QObject *parent)
    : QObject(parent)
    , serialHandler(serial)
    , isNavigating(false)
    , currentRotationAngle(0)
    , bestDirection(0)
    , rotationSpeed(100)
{
    navigationTimer = new QTimer(this);
    navigationTimer->setInterval(SCAN_INTERVAL);
    connect(navigationTimer, &QTimer::timeout, this, &AutonomousNav::onNavigationTimerTimeout);

    rotationTimer = new QTimer(this);
    rotationTimer->setInterval(50);  // 50ms dla płynnego obrotu
    connect(rotationTimer, &QTimer::timeout, this, &AutonomousNav::onRotationTimerTimeout);
}

void AutonomousNav::startNavigation() {
    if (!isNavigating) {
        isNavigating = true;
        navigationTimer->start();
        qDebug() << "Autonomous navigation started";
    }
}

void AutonomousNav::stopNavigation() {
    if (isNavigating) {
        isNavigating = false;
        navigationTimer->stop();
        rotationTimer->stop();
        serialHandler->sendMovementCommand('S');  // Stop
        qDebug() << "Autonomous navigation stopped";
    }
}

void AutonomousNav::processNavigationStep() {
    // Pobierz dane z wszystkich czujników
    auto rightSensor = serialHandler->sensorLastData['A'];  // Czujnik 1 (prawy)
    auto leftSensor = serialHandler->sensorLastData['B'];   // Czujnik 2 (lewy)
    auto frontSensor = serialHandler->sensorLastData['C'];  // Czujnik 3 (przód)

    if (rightSensor.isEmpty() || leftSensor.isEmpty() || frontSensor.isEmpty()) {
        qDebug() << "Waiting for sensor data...";
        return;
    }

    // Sprawdź czy bezpiecznie jest jechać do przodu
    if (isSafeToMove(frontSensor)) {
        // Dostosuj prędkość na podstawie odległości
        int frontDistance = frontSensor[4][4];  // Środkowy punkt
        adjustSpeed(frontDistance);
        serialHandler->sendMovementCommand('F');
    } else {
        // Zatrzymaj się i rozpocznij skanowanie otoczenia
        serialHandler->sendMovementCommand('S');
        startRotationScan();
    }
}

bool AutonomousNav::isSafeToMove(const QList<QList<int>>& sensorData) const {
    // Sprawdź środkowe elementy sensora
    for (int i = 3; i <= 5; i++) {
        for (int j = 3; j <= 5; j++) {
            if (sensorData[i][j] < SAFE_DISTANCE) {
                return false;
            }
        }
    }
    return true;
}

void AutonomousNav::startRotationScan() {
    currentRotationAngle = 0;
    bestDirection = 0;
    rotationTimer->start();
}

void AutonomousNav::processRotationStep() {
    auto rightSensor = serialHandler->sensorLastData['A'];
    auto leftSensor = serialHandler->sensorLastData['B'];
    auto frontSensor = serialHandler->sensorLastData['C'];

    if (rightSensor.isEmpty() || leftSensor.isEmpty() || frontSensor.isEmpty()) {
        return;
    }

    // Ocena kierunku
    int currentScore = evaluateDirection(frontSensor);
    if (currentScore > bestDirection) {
        bestDirection = currentRotationAngle;
    }

    // Kontynuuj obrót lub zakończ skanowanie
    if (currentRotationAngle < 360) {
        serialHandler->sendMovementCommand('R');
        currentRotationAngle += ROTATION_STEP;
    } else {
        rotationTimer->stop();
        // Obróć się w najlepszym znalezionym kierunku
        currentRotationAngle = bestDirection;
        serialHandler->sendMovementCommand('F');
        navigationTimer->start();
    }
}

int AutonomousNav::evaluateDirection(const QList<QList<int>>& sensorData) const {
    int score = 0;
    // Ocena na podstawie odległości w centralnym obszarze sensora
    for (int i = 2; i <= 6; i++) {
        for (int j = 2; j <= 6; j++) {
            if (sensorData[i][j] > SAFE_DISTANCE) {
                score += sensorData[i][j] - SAFE_DISTANCE;
            }
        }
    }
    return score;
}

void AutonomousNav::adjustSpeed(int distance) {
    int newSpeed;
    if (distance > SAFE_DISTANCE * 2) {
        newSpeed = 400;  // Pełna prędkość
    } else if (distance > SAFE_DISTANCE) {
        newSpeed = 250;  // Średnia prędkość
    } else {
        newSpeed = 150;  // Wolna prędkość
    }
    serialHandler->setSpeed(newSpeed);
}

void AutonomousNav::onNavigationTimerTimeout() {
    processNavigationStep();
}

void AutonomousNav::onRotationTimerTimeout() {
    processRotationStep();
}
