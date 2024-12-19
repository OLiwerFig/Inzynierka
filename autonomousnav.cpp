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
<<<<<<< Updated upstream
=======

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
            const double AZIMUTH_THRESHOLD = 0.01; // Próg zmiany w stopniach
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
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
void AutonomousNav::adjustSpeed(int distance) {
    int newSpeed;
    if (distance > SAFE_DISTANCE * 2) {
        newSpeed = 400;  // Pełna prędkość
    } else if (distance > SAFE_DISTANCE) {
        newSpeed = 250;  // Średnia prędkość
    } else {
        newSpeed = 150;  // Wolna prędkość
=======
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
            if (angle >= -5.0 && angle <= 5.0) {
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
        serialHandler->sendMovementCommand('F', BASE_SPEED, BASE_SPEED);
    } else {
        qDebug() << "Kontynuuję skręt w lewo. Szukam dobrego ustawienia.";
        // Kontynuacja skrętu w lewo
        serialHandler->sendMovementCommand('L', TURN_SPEED, TURN_SPEED);

>>>>>>> Stashed changes
    }
    serialHandler->setSpeed(newSpeed);
}

<<<<<<< Updated upstream
void AutonomousNav::onNavigationTimerTimeout() {
    processNavigationStep();
}

void AutonomousNav::onRotationTimerTimeout() {
    processRotationStep();
=======

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
        if (wallAngle < -5.0 || wallAngle > 5.0) {
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
                leftSpeed += static_cast<int>(-correction * 5);
            }

            leftSpeed = std::clamp(leftSpeed, 0, 400);
            rightSpeed = std::clamp(rightSpeed, 0, 400);

            // Używamy nowej wersji z dwoma prędkościami
            serialHandler->sendMovementCommand('F', leftSpeed, rightSpeed);

            qDebug() << "Śledzenie ściany - Azimuth:" << wallAngle
                     << "Korekcja:" << correction
                     << "Prędkości L/R:" << leftSpeed << "/" << rightSpeed;
        } else {
            serialHandler->sendMovementCommand('F', BASE_SPEED, BASE_SPEED);
            qDebug() << "Kąt ściany w zakresie docelowym.";
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
    serialHandler->sendMovementCommand('R', 150, 200);
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
>>>>>>> Stashed changes
}
