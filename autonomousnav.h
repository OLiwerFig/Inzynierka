#ifndef AUTONOMOUSNAV_H
#define AUTONOMOUSNAV_H

#include <QObject>
#include <QTimer>
#include "serialport.h"

class AutonomousNav : public QObject {
    Q_OBJECT

public:
    explicit AutonomousNav(serialport* serial, QObject *parent = nullptr);
    void startNavigation();
    void stopNavigation();

private:
    serialport* serialHandler;
    QTimer* navigationTimer;
    QTimer* rotationTimer;

    bool isNavigating;
    int currentRotationAngle;
    int bestDirection;
    int rotationSpeed;

    static const int SAFE_DISTANCE = 400;       // Bezpieczna odległość w mm
    static const int CRITICAL_DISTANCE = 200;   // Krytyczna odległość w mm
    static const int SCAN_INTERVAL = 100;       // Częstotliwość skanowania (ms)
    static const int ROTATION_STEP = 10;        // Krok obrotu w stopniach

    void processNavigationStep();
    void startRotationScan();
    void processRotationStep();
    int evaluateDirection(const QList<QList<int>>& sensorData) const;
    bool isSafeToMove(const QList<QList<int>>& sensorData) const;
    void adjustSpeed(int distance);

private slots:
    void onNavigationTimerTimeout();
    void onRotationTimerTimeout();
};

#endif // AUTONOMOUSNAV_H
