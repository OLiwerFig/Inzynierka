#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "draw.h"
#include "serialport.h"
#include "map.h"
#include <QSerialPortInfo>
#include <QGraphicsTextItem>
#include <QGraphicsPixmapItem>
#include <QDebug>
#include <QResizeEvent>
#include <QTranslator>
#include <QLibraryInfo>
#include <QTimer>

/**
 * \brief Konstruktor klasy MainWindow.
 *
 * Inicjalizuje interfejs użytkownika, obsługę portu szeregowego, widok i tłumacza.
 *
 * \param parent Wskaźnik na obiekt nadrzędny.
 */
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , serialPortHandler(new serialport(this))
    , setViewHandler(new SetView(this))
    , scene(nullptr)
    , gridPixmapItem(nullptr)
    , translator(new QTranslator(this))
    , map(nullptr)
    , isAutoNavActive(false)

{


    ui->setupUi(this);

    setViewHandler->setIcons(ui);  ///< Ustawia ikony w interfejsie użytkownika.
    setViewHandler->setTexts(ui, translator, this, false);  ///< Ustawia teksty w interfejsie użytkownika.
    setViewHandler->initializeLEDIndicator(ui);  ///< Inicjalizuje wskaźnik LED.
    Draw::initializeGraphicsScene(ui, scene, gridPixmapItem);  ///< Inicjalizuje scenę graficzną.





    if (!ui->MapGraphicsView) {
        qDebug() << "ui->MapGraphicsView jest nullptr!";
        return;
    }

    map = new Map(ui->MapGraphicsView);
    map->initializeMap("/Users/oliwerfigura/Desktop/Inzynierka/Inzynierka/obstacles.txt");

    setFocusPolicy(Qt::StrongFocus);
    ui->MapGraphicsView->setFocusPolicy(Qt::StrongFocus); //
    connect(ui->upButton, &QPushButton::pressed, this, [this]() {
        serialPortHandler->sendMovementCommand('F');
    });
    connect(ui->downButton, &QPushButton::pressed, this, [this]() {
        serialPortHandler->sendMovementCommand('B');
    });
    connect(ui->leftButton, &QPushButton::pressed, this, [this]() {
        serialPortHandler->sendMovementCommand('L');
    });
    connect(ui->rightButton, &QPushButton::pressed, this, [this]() {
        serialPortHandler->sendMovementCommand('R');
    });

    // Połączenia dla puszczenia przycisków
    connect(ui->upButton, &QPushButton::released, this, [this]() {
        serialPortHandler->sendMovementCommand('S');
    });
    connect(ui->downButton, &QPushButton::released, this, [this]() {
        serialPortHandler->sendMovementCommand('S');
    });
    connect(ui->leftButton, &QPushButton::released, this, [this]() {
        serialPortHandler->sendMovementCommand('S');
    });
    connect(ui->rightButton, &QPushButton::released, this, [this]() {
        serialPortHandler->sendMovementCommand('S');
    });



    ui->MapGraphicsView->setMinimumSize(600, 600);
    ui->graphicsView->setMinimumSize(600, 600);

    ui->AngleLabel->setMinimumSize(600, 200);
    ui->AngleLabel->setMinimumSize(600, 200);

    ui->FlagiComboBox->addItem("Czujnik 1");
    ui->FlagiComboBox->addItem("Czujnik 2");
    ui->FlagiComboBox->addItem("Czujnik 3");

    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(1000);
    ui->horizontalSlider->setValue(400); // wartość początkowa
    ui->SpeedLabel->setText("Speed: 400");

    // Połączenie sygnału zmiany indeksu z funkcją obsługującą wysyłanie flagi
    connect(ui->FlagiComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onFlagChanged()));

    serialPortHandler->populateAvailablePorts(ui);  ///< Wypełnia listę dostępnych portów szeregowych.

    // Połączenie sygnałów i slotów
    connect(ui->buttonConnect, &QPushButton::clicked, this, [this]() {
        serialPortHandler->connectSerialPort(ui->comboBoxSerialPorts->currentText());
    });
    connect(ui->refreshButton, &QPushButton::clicked, this, [this]() {
        serialPortHandler->refreshConnection(ui);
    });

    connect(serialPortHandler, &serialport::serialDataReceived, this, [this](const QList<QByteArray> &data) {
        serialPortHandler->handleSerialData(this, ui, scene, data);
    });

    connect(serialPortHandler, &serialport::portStatusChanged, this, [this](bool connected, const QString &portName) {
        serialPortHandler->handlePortStatusChanged(ui, connected, portName);
    });
    connect(ui->languageButton, &QPushButton::clicked, this, [this]() {
        setViewHandler->toggleLanguage(ui, translator, this);
    });

    connect(ui->horizontalSlider, &QSlider::valueChanged, this, [this](int value) {
        qDebug() << "Slider zmieniony na:" << value;
        ui->SpeedLabel->setText(QString("Speed: %1").arg(value));
        serialPortHandler->setSpeed(value);
    });


    autoNav = new AutonomousNav(serialPortHandler, ui->directionLabel, this);

    // Połączenie przycisku autoNavButton z nowym slotem
    connect(ui->autoNavButton, &QPushButton::clicked, this, &MainWindow::toggleAutoNavigation);



    serialPortHandler->connectSerialPort(ui->comboBoxSerialPorts->currentText());

    connect(serialPortHandler, &serialport::activeSensorCountChanged,
            this, &MainWindow::updateSensorCount);

    // Inicjalizacja etykiety
    ui->activeSensorsLabel->setText("Sensors: 0/3");

}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatioByExpanding);
    if (map) {
        map->scaleSceneToFitView();
    }
}

void MainWindow::toggleAutoNavigation() {
    isAutoNavActive = !isAutoNavActive;

    if (isAutoNavActive) {
        ui->autoNavButton->setText("Stop");
        autoNav->startNavigation();

        // Wyłączenie manualnych przycisków (opcjonalnie)
        // Jeśli chcesz, aby manualne kontrole były nadal dostępne, usuń poniższe linie
        /*
        ui->upButton->setEnabled(false);
        ui->downButton->setEnabled(false);
        ui->leftButton->setEnabled(false);
        ui->rightButton->setEnabled(false);
        */
    } else {
        ui->autoNavButton->setText("Start");
        autoNav->stopNavigation();

        // Włączenie manualnych przycisków (opcjonalnie)
        /*
        ui->upButton->setEnabled(true);
        ui->downButton->setEnabled(true);
        ui->leftButton->setEnabled(true);
        ui->rightButton->setEnabled(true);
        */
    }
}


void MainWindow::updateSensorCount(int count) {
    if (ui->activeSensorsLabel) {
        ui->activeSensorsLabel->setText(QString("Sensors: %1/3").arg(count));
    }
}


bool isUpPressed = false;
bool isDownPressed = false;
bool isLeftPressed = false;
bool isRightPressed = false;

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (!map) return;

    switch (event->key()) {
    case Qt::Key_Up:
    case Qt::Key_W:
        map->moveRobotUp();
        serialPortHandler->sendMovementCommand('F'); // Forward
        isUpPressed = true;
        break;
    case Qt::Key_Down:
    case Qt::Key_S:
        map->moveRobotDown();
        serialPortHandler->sendMovementCommand('B'); // Backward
        isDownPressed = true;
        break;
    case Qt::Key_Left:
    case Qt::Key_A:
        map->moveRobotLeft();
        serialPortHandler->sendMovementCommand('L'); // Left
        isLeftPressed = true;
        break;
    case Qt::Key_Right:
    case Qt::Key_D:
        map->moveRobotRight();
        serialPortHandler->sendMovementCommand('R'); // Right
        isRightPressed = true;
        break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *event) {
    switch (event->key()) {
    case Qt::Key_Up:
    case Qt::Key_W:
        serialPortHandler->sendMovementCommand('S'); // Stop
        isUpPressed = false;
        break;
    case Qt::Key_Down:
    case Qt::Key_S:
        serialPortHandler->sendMovementCommand('S'); // Stop
        isDownPressed = false;
        break;
    case Qt::Key_Left:
    case Qt::Key_A:
        serialPortHandler->sendMovementCommand('S'); // Stop
        isLeftPressed = false;
        break;
    case Qt::Key_Right:
    case Qt::Key_D:
        serialPortHandler->sendMovementCommand('S'); // Stop
        isRightPressed = false;
        break;
    }
}

void MainWindow::timerEvent(QTimerEvent *event) {
    if (isUpPressed) {
        map->moveRobotUp();
        serialPortHandler->sendMovementCommand('F'); // Forward
    }
    if (isDownPressed) {
        map->moveRobotDown();
        serialPortHandler->sendMovementCommand('B'); // Backward
    }
    if (isLeftPressed) {
        map->moveRobotLeft();
        serialPortHandler->sendMovementCommand('L'); // Left
    }
    if (isRightPressed) {
        map->moveRobotRight();
        serialPortHandler->sendMovementCommand('R'); // Right
    }
}

void MainWindow::onSpeedSliderChanged(int value) {
    // Aktualizuj etykietę
    ui->SpeedLabel->setText(QString("Speed: %1").arg(value));
    // Aktualizuj prędkość w serialport handler
    serialPortHandler->setSpeed(value);
}

void MainWindow::initializeMap() {
    if (!ui->MapGraphicsView) {
        qDebug() << "ui->MapGraphicsView nadal jest nullptr!";
        return;
    }

    map = new Map(ui->MapGraphicsView);
    qDebug() << "Obiekt Map zainicjalizowany!";
    map->loadObstacles("/Users/oliwerfigura/Desktop/Inzynierka/Inzynierka/obstacles.txt");
    qDebug() << "Przeszkody załadowane!";
}


void MainWindow::onFlagChanged() {
    // Wywołanie funkcji wysyłania flagi
    serialPortHandler->sendFlag(ui);
}

/**
 * \brief Destruktor klasy MainWindow.
 *
 * Usuwa interfejs użytkownika.
 */
MainWindow::~MainWindow()
{
    delete map;
    delete ui;
}
