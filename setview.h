#ifndef SETVIEW_H
#define SETVIEW_H

#include <QWidget>
#include <QTranslator>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QIcon>
#include "ui_mainwindow.h"

class SetView : public QObject
{
    Q_OBJECT

public:
    SetView(QObject *parent = nullptr);
    void setIcons(Ui::MainWindow *ui);
    void setTexts(Ui::MainWindow *ui, QTranslator *translator, QMainWindow *mainWindow, bool isEnglish);
    void toggleLanguage(Ui::MainWindow *ui, QTranslator *translator, QMainWindow *mainWindow);
    void initializeLEDIndicator(Ui::MainWindow *ui);

private:
    void updateTexts(Ui::MainWindow *ui);
};

#endif // SETVIEW_H
