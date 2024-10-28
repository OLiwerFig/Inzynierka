/**
 * \file setview.cpp
 * \brief Implementacja klasy SetView.
 */

#include "setview.h"
#include <QDebug>

/**
 * \brief Konstruktor klasy SetView.
 *
 * Inicjalizuje klasę bazową QObject oraz przygotowuje instancję SetView do użycia.
 *
 * \param parent Wskaźnik na obiekt nadrzędny, zwykle główne okno aplikacji.
 */
SetView::SetView(QObject *parent) : QObject(parent) {}

/**
 * \brief Ustawia ikony dla przycisków w interfejsie użytkownika.
 *
 * Ustawia ikony przycisków w głównym oknie aplikacji na podstawie zasobów graficznych.
 *
 * \param ui Wskaźnik do interfejsu użytkownika klasy MainWindow, używany do dostępu do przycisków.
 */
void SetView::setIcons(Ui::MainWindow *ui)
{
    QIcon refreshIcon(":/images/retry.png");
    ui->refreshButton->setIcon(refreshIcon);
    ui->refreshButton->setIconSize(QSize(24, 24));

    QIcon conIcon(":/images/con.png");
    ui->buttonConnect->setIcon(conIcon);
    ui->buttonConnect->setIconSize(QSize(32, 32));

    ui->languageButton->setIcon(QIcon(":/images/pol.png"));
    ui->languageButton->setIconSize(QSize(32, 32));
}

/**
 * \brief Ustawia teksty dla interfejsu użytkownika na podstawie wybranego języka.
 *
 * Ładuje pliki tłumaczeń i stosuje je do interfejsu użytkownika. Obsługuje przełączanie między językami angielskim i polskim.
 *
 * \param ui Wskaźnik do interfejsu użytkownika.
 * \param translator Wskaźnik do tłumacza Qt, używany do zmiany języków.
 * \param mainWindow Wskaźnik do głównego okna aplikacji, który jest używany do ponownego tłumaczenia interfejsu.
 * \param isEnglish Flaga określająca, czy język angielski jest obecnie używany.
 */
void SetView::setTexts(Ui::MainWindow *ui, QTranslator *translator, QMainWindow *mainWindow, bool isEnglish)
{
    bool loaded = false;
    if (isEnglish) {
        loaded = translator->load(":/translations/app_en.qm");
        ui->languageButton->setIcon(QIcon(":/images/pol.png"));
    } else {
        loaded = translator->load(":/translations/app_pl.qm");
        ui->languageButton->setIcon(QIcon(":/images/ang.png"));
    }

    if (!loaded) {
        qDebug() << "Failed to load translation file.";
    } else {
        qApp->installTranslator(translator);
        ui->retranslateUi(mainWindow);
        updateTexts(ui);
    }
}

/**
 * \brief Przełącza język interfejsu użytkownika.
 *
 * Zmienia język interfejsu użytkownika między angielskim a polskim. Aktualizuje tłumaczenia interfejsu.
 *
 * \param ui Wskaźnik do interfejsu użytkownika.
 * \param translator Wskaźnik do tłumacza Qt.
 * \param mainWindow Wskaźnik do głównego okna aplikacji.
 */
void SetView::toggleLanguage(Ui::MainWindow *ui, QTranslator *translator, QMainWindow *mainWindow)
{
    static bool isEnglish = false;
    isEnglish = !isEnglish;
    setTexts(ui, translator, mainWindow, isEnglish);
}


/**
 * \brief Inicjalizuje wskaźnik LED.
 *
 * Ustawia wskaźnik LED na czerwony, sygnalizując brak połączenia, oraz aktualizuje tekst statusu.
 *
 * \param ui Wskaźnik do interfejsu użytkownika, który jest używany do dostępu do wskaźnika LED i etykiety statusu.
 */
void SetView::initializeLEDIndicator(Ui::MainWindow *ui)
{
    ui->ledIndicator->setStyleSheet("background-color: red; border-radius: 10px;");
    ui->statusLabel->setText(QObject::tr("Disconnected"));
}

/**
 * \brief Aktualizuje teksty w interfejsie użytkownika.
 *
 * Aktualizuje teksty na interfejsie użytkownika po zmianie języka lub inicjalizacji.
 *
 * \param ui Wskaźnik do interfejsu użytkownika.
 */
void SetView::updateTexts(Ui::MainWindow *ui)
{
    ui->speed_text_L->setText(QObject::tr("Prędkość lewego silnika"));
    ui->speed_text_R->setText(QObject::tr("Prędkość prawego silnika"));
    ui->pwn_text_R->setText(QObject::tr("Wartość wypełnienia PWM prawego silnika"));
    ui->pwn_text_L->setText(QObject::tr("Wartość wypełnienia PWM lewego silnika"));
    ui->actualTarget->setText(QObject::tr("Aktualny Target"));
    ui->actual_position->setText(QObject::tr("Aktualna Pozycja"));
    ui->send_target->setText(QObject::tr("Wpisz target"));
    ui->buttonSendTarget->setText(QObject::tr("Send"));
    ui->statusLabel->setText(QObject::tr("Disconnected"));

}
