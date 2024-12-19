#ifndef AUTONOMOUSNAV_H
#define AUTONOMOUSNAV_H

#include <QObject>
#include <QDateTime>
#include "multiplanedetector.h"
#include <QLabel>
#include "serialport.h"
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QDateTime>


class PIDController {
public:
    PIDController(double kp = 0.8, double ki = 0.1, double kd = 0.3);
    double calculate(double setpoint, double currentValue, double deltaTime);
    void reset();
private:
    double kp, ki, kd;
    double previousError;
    double integral;
};

class AutonomousNav : public QObject {
    Q_OBJECT
public:
    explicit AutonomousNav(serialport* serial, QLabel* label, QObject *parent = nullptr);
    void startNavigation();
    void stopNavigation();

private:




    QString logsDirectory;        // Katalog na pliki logów
    QString currentLogFileName;   // Nazwa aktualnego pliku logów
    QFile logFile;               // Plik logów
    QTextStream logStream;       // Strumień tekstowy do zapisu
    double lastAzimuthB;         // Ostatni znany kąt azymutu dla czujnika B

    void initializeLogging();    // Nowa metoda do inicjalizacji logowania




    // Komponenty
    serialport* serialHandler;
    PIDController* wallPID;
    QLabel* directionLabel;  // Label do wyświetlania poleceń

    // Zmienne stanu
    bool isNavigating;
    bool isCurrentlyTurning;
    qint64 lastUpdateTime;
    QList<QString> lastCommands;  // Bufor ostatnich poleceń

    // Stałe konfiguracyjne
    static constexpr int SAFE_DISTANCE = 120;          // mm
    static constexpr int FRONT_WALL_DISTANCE = 100;    // mm
    static constexpr int BASE_SPEED = 400;             // Prędkość podstawowa
    static constexpr int TURN_SPEED = 300;             // Prędkość skrętu

    // Mapowanie komend
    const QMap<char, QString> commandDescriptions = {
        {'F', "Forward"},
        {'B', "Backward"},
        {'L', "Left"},
        {'R', "Right"},
        {'S', "Stop"}
    };

    // Śledzenie ostatniego wysłanego polecenia ruchu
    char lastMovementCommand;

    // Funkcje nawigacji
    void processNavigationStep();
    void followWallSequence(const QList<QList<int>>& leftSensor,
                            const QList<QList<int>>& rightSensor);
    void handleTurningSequence(const QList<QList<int>>& leftSensor,
                               const QList<QList<int>>& rightSensor,
                               const QList<QList<int>>& frontSensor);

    // Funkcje pomocnicze
    bool checkFrontCollisionRisk(const QList<QList<int>>& frontSensor);
    void handleFrontWall();
    void searchForWall();
    void updateDirectionLabel(char command);

    // Funkcja pomocnicza do wysyłania poleceń ruchu
    void sendMovementCommand(char command);

    QTimer* decelerationTimer;
    int currentSpeed;
    int decelerationStep;
    int minimumSpeed;

    // Metoda pomocnicza do dekrementacji prędkości
    void performDeceleration();



public slots:
    void onNewSensorData();
};

#endif // AUTONOMOUSNAV_H
