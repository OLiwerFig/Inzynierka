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
    , map(nullptr)
    , translator(new QTranslator(this))

{


    ui->setupUi(this);

    setViewHandler->setIcons(ui);  ///< Ustawia ikony w interfejsie użytkownika.
    setViewHandler->setTexts(ui, translator, this, false);  ///< Ustawia teksty w interfejsie użytkownika.
    setViewHandler->initializeLEDIndicator(ui);  ///< Inicjalizuje wskaźnik LED.
    Draw::initializeGraphicsScene(ui, scene, gridPixmapItem);  ///< Inicjalizuje scenę graficzną.

    setFocusPolicy(Qt::StrongFocus);
    ui->MapGraphicsView->setFocusPolicy(Qt::StrongFocus); //
    connect(ui->upButton, &QPushButton::clicked, map, &Map::moveRobotUp);
    connect(ui->downButton, &QPushButton::clicked, map, &Map::moveRobotDown);
    connect(ui->leftButton, &QPushButton::clicked, map, &Map::moveRobotLeft);
    connect(ui->rightButton, &QPushButton::clicked, map, &Map::moveRobotRight);




    if (!ui->MapGraphicsView) {
        qDebug() << "ui->MapGraphicsView jest nullptr!";
        return;
    }

    map = new Map(ui->MapGraphicsView);
    map->initializeMap("/Users/oliwerfigura/Desktop/Inzynierka/Inzynierka/obstacles.txt");


    ui->MapGraphicsView->setMinimumSize(600, 600);

    ui->FlagiComboBox->addItem("Czujnik 1");
    ui->FlagiComboBox->addItem("Czujnik 2");
    ui->FlagiComboBox->addItem("Czujnik 3");

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
    connect(ui->buttonSendTarget, &QPushButton::clicked, this, [this]() {
        serialPortHandler->sendTargetCoordinates(ui);
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


    serialPortHandler->connectSerialPort(ui->comboBoxSerialPorts->currentText());
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatioByExpanding);
    if (map) {
        map->scaleSceneToFitView();
    }
}


void MainWindow::keyPressEvent(QKeyEvent *event) {
    if (!map) return;

    switch (event->key()) {
    case Qt::Key_Up:
    case Qt::Key_W:
        map->moveRobotUp();
        break;
    case Qt::Key_Down:
    case Qt::Key_S:
        map->moveRobotDown();
        break;
    case Qt::Key_Left:
    case Qt::Key_A:
        map->moveRobotLeft();
        break;
    case Qt::Key_Right:
    case Qt::Key_D:
        map->moveRobotRight();
        break;
    }
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
