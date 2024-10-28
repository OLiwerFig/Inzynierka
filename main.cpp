/**
 * \file main.cpp
 * \brief Główny plik aplikacji uruchamiający okno główne.
 */

#include "mainwindow.h"
#include <QApplication>

/**
 * \brief Główna funkcja aplikacji.
 *
 * Funkcja inicjalizuje aplikację, tworzy główne okno i uruchamia pętlę zdarzeń Qt.
 *
 * \param argc Liczba argumentów wiersza poleceń.
 * \param argv Tablica argumentów wiersza poleceń.
 * \return Zwraca kod zakończenia aplikacji.
 */
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);  ///< Tworzy instancję aplikacji Qt.
    MainWindow w;                ///< Tworzy główne okno aplikacji.
    w.show();                    ///< Wyświetla główne okno aplikacji.
    return a.exec();             ///< Uruchamia pętlę zdarzeń aplikacji.
}
