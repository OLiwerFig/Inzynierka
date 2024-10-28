/**
 * \file mainwindow.cpp
 * \brief Implementacja klasy MainWindow.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "draw.h"
#include "serialport.h"
#include <QSerialPortInfo>
#include <QGraphicsTextItem>
#include <QGraphicsPixmapItem>
#include <QDebug>
#include <QResizeEvent>
#include <QTranslator>
#include <QLibraryInfo>

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
    , translator(new QTranslator(this))
{
    ui->setupUi(this);

    setViewHandler->setIcons(ui);  ///< Ustawia ikony w interfejsie użytkownika.
    setViewHandler->setTexts(ui, translator, this, false);  ///< Ustawia teksty w interfejsie użytkownika.
    setViewHandler->initializeLEDIndicator(ui);  ///< Inicjalizuje wskaźnik LED.
    Draw::initializeGraphicsScene(ui, scene, gridPixmapItem);  ///< Inicjalizuje scenę graficzną.

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



    // Próba otwarcia portu szeregowego przy starcie
    serialPortHandler->connectSerialPort(ui->comboBoxSerialPorts->currentText());
}

void MainWindow::resizeEvent(QResizeEvent *event) {
    QMainWindow::resizeEvent(event);
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatioByExpanding);
}



/**
 * \brief Destruktor klasy MainWindow.
 *
 * Usuwa interfejs użytkownika.
 */
MainWindow::~MainWindow()
{
    delete ui;
}
