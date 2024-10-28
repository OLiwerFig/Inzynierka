/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout_6;
    QHBoxLayout *main;
    QVBoxLayout *verticalLayout_7;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_5;
    QLabel *ledIndicator;
    QLabel *statusLabel;
    QPushButton *refreshButton;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *languageButton;
    QComboBox *comboBoxSerialPorts;
    QPushButton *buttonConnect;
    QVBoxLayout *verticalLayout_6;
    QGraphicsView *graphicsView;
    QPushButton *clearButton;
    QVBoxLayout *verticalLayout_4;
    QVBoxLayout *verticalLayout;
    QLabel *pwn_text_L;
    QProgressBar *pwmLProgressBar;
    QLabel *pwn_text_R;
    QProgressBar *pwmRProgressBar;
    QLabel *speed_text_L;
    QProgressBar *speedLProgressBar;
    QLabel *speed_text_R;
    QProgressBar *speedRProgressBar;
    QVBoxLayout *verticalLayout_3;
    QLabel *actual_position;
    QHBoxLayout *horizontalLayout_2;
    QLabel *labelPosY;
    QLabel *labelPosX;
    QLabel *labelTheta;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *actualTarget;
    QLabel *send_target;
    QHBoxLayout *horizontalLayout;
    QLabel *labelTargetX;
    QLabel *labelTargetY;
    QLineEdit *lineEditTargetX;
    QLineEdit *lineEditTargetY;
    QPushButton *buttonSendTarget;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(962, 534);
        MainWindow->setStyleSheet(QString::fromUtf8("QMainWindow {\n"
"    background-color: #F5EFE6; /* Bia\305\202e t\305\202o dla ca\305\202ego okna */\n"
"    border: 2px solid #1A4D2E; /* Kolor ramki pasuj\304\205cy do reszty */\n"
"    border-radius: 10px; /* Zaokr\304\205glone rogi */\n"
"}"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        horizontalLayout_6 = new QHBoxLayout(centralwidget);
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        main = new QHBoxLayout();
        main->setObjectName("main");
        main->setSizeConstraint(QLayout::SetNoConstraint);
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName("verticalLayout_7");
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName("verticalLayout_5");
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        ledIndicator = new QLabel(centralwidget);
        ledIndicator->setObjectName("ledIndicator");

        horizontalLayout_5->addWidget(ledIndicator);

        statusLabel = new QLabel(centralwidget);
        statusLabel->setObjectName("statusLabel");
        statusLabel->setStyleSheet(QString::fromUtf8("QLabel#statusLabel {\n"
"    border: 2px solid #4F6F52; \n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"    background-color: white; \n"
"    color: black;\n"
"}\n"
"\n"
"QLabel#statusLabel[connected=\"true\"] {\n"
"    color: green;\n"
"    font-weight: bold; \n"
"}\n"
"\n"
"QLabel#statusLabel[connected=\"false\"] {\n"
"    color: red;\n"
"    font-weight: bold;\n"
"}\n"
""));

        horizontalLayout_5->addWidget(statusLabel);

        refreshButton = new QPushButton(centralwidget);
        refreshButton->setObjectName("refreshButton");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(refreshButton->sizePolicy().hasHeightForWidth());
        refreshButton->setSizePolicy(sizePolicy);
        refreshButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    padding: 0px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QPushButton#refreshButton {\n"
"    background-color: white;\n"
"}\n"
"\n"
"QPushButton#refreshButton:pressed {\n"
"    background-color: #4F6F52;\n"
"}\n"
""));

        horizontalLayout_5->addWidget(refreshButton);

        horizontalLayout_5->setStretch(0, 1);
        horizontalLayout_5->setStretch(1, 6);
        horizontalLayout_5->setStretch(2, 1);

        verticalLayout_5->addLayout(horizontalLayout_5);

        horizontalLayout_4 = new QHBoxLayout();
#ifndef Q_OS_MAC
        horizontalLayout_4->setSpacing(-1);
#endif
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        horizontalLayout_4->setSizeConstraint(QLayout::SetDefaultConstraint);
        languageButton = new QPushButton(centralwidget);
        languageButton->setObjectName("languageButton");
        sizePolicy.setHeightForWidth(languageButton->sizePolicy().hasHeightForWidth());
        languageButton->setSizePolicy(sizePolicy);
        languageButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    padding: 0px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    background-position: center;\n"
"    background-repeat: no-repeat;\n"
"    background-size: contain;\n"
"}\n"
"\n"
"QPushButton#languageButton[icon='pol'] {\n"
"    background-image: url(:/images/pol.png);\n"
"}\n"
"\n"
"QPushButton#languageButton[icon='eng'] {\n"
"    background-image: url(:/images/ang.png);\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: #4F6F52;\n"
"}\n"
""));

        horizontalLayout_4->addWidget(languageButton);

        comboBoxSerialPorts = new QComboBox(centralwidget);
        comboBoxSerialPorts->setObjectName("comboBoxSerialPorts");
        comboBoxSerialPorts->setStyleSheet(QString::fromUtf8("QComboBox {\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"    background-color: white;\n"
"    color: black;\n"
"    selection-background-color: #4CAF50;\n"
"    selection-color: white;\n"
"}\n"
"\n"
"QComboBox QAbstractItemView {\n"
"    border: 2px solid #4F6F52;\n"
"    background-color: white;\n"
"    selection-background-color: #4CAF50;\n"
"    selection-color: white;\n"
"}\n"
"\n"
"QComboBox::drop-down {\n"
"    subcontrol-origin: padding;\n"
"    subcontrol-position: top right;\n"
"    width: 20px;\n"
"    border-left-width: 1px;\n"
"    border-left-color: #4F6F52;\n"
"    border-left-style: solid;\n"
"    border-top-right-radius: 5px;\n"
"    border-bottom-right-radius: 5px;\n"
"    background-color: #4F6F52;\n"
"}\n"
"\n"
"QComboBox::down-arrow {\n"
"    image: url(:/icons/down_arrow.png);\n"
"    width: 10px;\n"
"    height: 10px;\n"
"}\n"
""));

        horizontalLayout_4->addWidget(comboBoxSerialPorts);

        buttonConnect = new QPushButton(centralwidget);
        buttonConnect->setObjectName("buttonConnect");
        sizePolicy.setHeightForWidth(buttonConnect->sizePolicy().hasHeightForWidth());
        buttonConnect->setSizePolicy(sizePolicy);
        buttonConnect->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    padding: 0px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    background-position: center;\n"
"    background-repeat: no-repeat;\n"
"    background-size: contain;\n"
"}\n"
"\n"
"QPushButton#buttonConnect {\n"
"    background-color: white;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QPushButton#buttonConnect:pressed {\n"
"    background-color: #4F6F52;\n"
"}\n"
""));

        horizontalLayout_4->addWidget(buttonConnect);

        horizontalLayout_4->setStretch(0, 1);
        horizontalLayout_4->setStretch(1, 6);
        horizontalLayout_4->setStretch(2, 1);

        verticalLayout_5->addLayout(horizontalLayout_4);

        verticalLayout_5->setStretch(0, 1);
        verticalLayout_5->setStretch(1, 1);

        verticalLayout_7->addLayout(verticalLayout_5);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName("verticalLayout_6");
        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName("graphicsView");
        sizePolicy.setHeightForWidth(graphicsView->sizePolicy().hasHeightForWidth());
        graphicsView->setSizePolicy(sizePolicy);
        graphicsView->setMinimumSize(QSize(200, 200));
        graphicsView->setMaximumSize(QSize(2000, 2000));
        graphicsView->setSizeIncrement(QSize(1000, 1000));
        graphicsView->setStyleSheet(QString::fromUtf8("QGraphicsView {\n"
"    border: 2px solid #4F6F52; /* Kolor ramki pasuj\304\205cy do reszty */\n"
"    border-radius: 5px; /* Zaokr\304\205glone rogi */\n"
"    background-color: white; /* Jasnozielone t\305\202o */\n"
"}\n"
""));

        verticalLayout_6->addWidget(graphicsView);

        clearButton = new QPushButton(centralwidget);
        clearButton->setObjectName("clearButton");
        clearButton->setEnabled(true);
        sizePolicy.setHeightForWidth(clearButton->sizePolicy().hasHeightForWidth());
        clearButton->setSizePolicy(sizePolicy);
        clearButton->setAcceptDrops(false);
        clearButton->setLayoutDirection(Qt::LeftToRight);
        clearButton->setAutoFillBackground(false);
        clearButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    padding: 0px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    background-position: center;\n"
"    background-repeat: no-repeat;\n"
"    background-size: contain;\n"
"}\n"
"\n"
"QPushButton#clearButton {\n"
"    background-color: white;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QPushButton#clearButton:pressed {\n"
"    background-color: #4F6F52;\n"
"    color: white; \n"
"}\n"
""));

        verticalLayout_6->addWidget(clearButton);

        verticalLayout_6->setStretch(0, 15);
        verticalLayout_6->setStretch(1, 1);

        verticalLayout_7->addLayout(verticalLayout_6);

        verticalLayout_7->setStretch(0, 1);
        verticalLayout_7->setStretch(1, 4);

        main->addLayout(verticalLayout_7);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName("verticalLayout_4");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        pwn_text_L = new QLabel(centralwidget);
        pwn_text_L->setObjectName("pwn_text_L");
        QFont font;
        font.setFamilies({QString::fromUtf8("Arial")});
        font.setPointSize(18);
        pwn_text_L->setFont(font);
        pwn_text_L->setStyleSheet(QString::fromUtf8(""));

        verticalLayout->addWidget(pwn_text_L);

        pwmLProgressBar = new QProgressBar(centralwidget);
        pwmLProgressBar->setObjectName("pwmLProgressBar");
        pwmLProgressBar->setAutoFillBackground(false);
        pwmLProgressBar->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 2px solid grey;\n"
"    border-radius: 5px;\n"
"    text-align: center;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        x1: 0, y1: 0, x2: 1, y2: 0,\n"
"        stop: 0 #05B8CC, stop: 1 #3DCC3D\n"
"    );\n"
"    width: 20px;\n"
"}\n"
""));
        pwmLProgressBar->setMaximum(400);
        pwmLProgressBar->setValue(0);
        pwmLProgressBar->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        pwmLProgressBar->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(pwmLProgressBar);

        pwn_text_R = new QLabel(centralwidget);
        pwn_text_R->setObjectName("pwn_text_R");
        pwn_text_R->setFont(font);

        verticalLayout->addWidget(pwn_text_R);

        pwmRProgressBar = new QProgressBar(centralwidget);
        pwmRProgressBar->setObjectName("pwmRProgressBar");
        pwmRProgressBar->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 2px solid grey;\n"
"    border-radius: 5px;\n"
"    text-align: center;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        x1: 0, y1: 0, x2: 1, y2: 0,\n"
"        stop: 0 #05B8CC, stop: 1 #3DCC3D\n"
"    );\n"
"    width: 20px;\n"
"}\n"
""));
        pwmRProgressBar->setMaximum(400);
        pwmRProgressBar->setValue(0);
        pwmRProgressBar->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        pwmRProgressBar->setInvertedAppearance(false);

        verticalLayout->addWidget(pwmRProgressBar);

        speed_text_L = new QLabel(centralwidget);
        speed_text_L->setObjectName("speed_text_L");
        speed_text_L->setFont(font);

        verticalLayout->addWidget(speed_text_L);

        speedLProgressBar = new QProgressBar(centralwidget);
        speedLProgressBar->setObjectName("speedLProgressBar");
        speedLProgressBar->setAutoFillBackground(false);
        speedLProgressBar->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 2px solid grey;\n"
"    border-radius: 5px;\n"
"    text-align: center;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        x1: 0, y1: 0, x2: 1, y2: 0,\n"
"        stop: 0 #05B8CC, stop: 1 #3DCC3D\n"
"    );\n"
"    width: 20px;\n"
"}\n"
""));
        speedLProgressBar->setMaximum(120);
        speedLProgressBar->setValue(0);
        speedLProgressBar->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);
        speedLProgressBar->setTextVisible(true);
        speedLProgressBar->setInvertedAppearance(false);
        speedLProgressBar->setTextDirection(QProgressBar::TopToBottom);

        verticalLayout->addWidget(speedLProgressBar);

        speed_text_R = new QLabel(centralwidget);
        speed_text_R->setObjectName("speed_text_R");
        speed_text_R->setFont(font);

        verticalLayout->addWidget(speed_text_R);

        speedRProgressBar = new QProgressBar(centralwidget);
        speedRProgressBar->setObjectName("speedRProgressBar");
        speedRProgressBar->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 2px solid grey;\n"
"    border-radius: 5px;\n"
"    text-align: center;\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        x1: 0, y1: 0, x2: 1, y2: 0,\n"
"        stop: 0 #05B8CC, stop: 1 #3DCC3D\n"
"    );\n"
"    width: 20px;\n"
"}\n"
""));
        speedRProgressBar->setMaximum(120);
        speedRProgressBar->setValue(0);
        speedRProgressBar->setAlignment(Qt::AlignJustify|Qt::AlignVCenter);

        verticalLayout->addWidget(speedRProgressBar);


        verticalLayout_4->addLayout(verticalLayout);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName("verticalLayout_3");
        actual_position = new QLabel(centralwidget);
        actual_position->setObjectName("actual_position");
        actual_position->setFont(font);
        actual_position->setStyleSheet(QString::fromUtf8(""));

        verticalLayout_3->addWidget(actual_position);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        labelPosY = new QLabel(centralwidget);
        labelPosY->setObjectName("labelPosY");
        QFont font1;
        font1.setFamilies({QString::fromUtf8("Arial")});
        labelPosY->setFont(font1);
        labelPosY->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    padding: 5px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    color: black; /* Kolor tekstu */\n"
"    font-size: 16px; /* Wi\304\231kszy rozmiar czcionki */\n"
"}\n"
""));

        horizontalLayout_2->addWidget(labelPosY);

        labelPosX = new QLabel(centralwidget);
        labelPosX->setObjectName("labelPosX");
        labelPosX->setFont(font1);
        labelPosX->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    padding: 5px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    color: black; /* Kolor tekstu */\n"
"    font-size: 16px; /* Wi\304\231kszy rozmiar czcionki */\n"
"}\n"
""));

        horizontalLayout_2->addWidget(labelPosX);

        labelTheta = new QLabel(centralwidget);
        labelTheta->setObjectName("labelTheta");
        labelTheta->setFont(font1);
        labelTheta->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    padding: 5px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    color: black; /* Kolor tekstu */\n"
"    font-size: 16px; /* Wi\304\231kszy rozmiar czcionki */\n"
"}\n"
""));
        labelTheta->setLineWidth(1);

        horizontalLayout_2->addWidget(labelTheta);


        verticalLayout_3->addLayout(horizontalLayout_2);

        verticalLayout_3->setStretch(1, 4);

        verticalLayout_4->addLayout(verticalLayout_3);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        actualTarget = new QLabel(centralwidget);
        actualTarget->setObjectName("actualTarget");
        actualTarget->setFont(font);

        horizontalLayout_3->addWidget(actualTarget);

        send_target = new QLabel(centralwidget);
        send_target->setObjectName("send_target");
        send_target->setFont(font);

        horizontalLayout_3->addWidget(send_target);

        horizontalLayout_3->setStretch(0, 2);
        horizontalLayout_3->setStretch(1, 3);

        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        labelTargetX = new QLabel(centralwidget);
        labelTargetX->setObjectName("labelTargetX");
        labelTargetX->setFont(font1);
        labelTargetX->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    padding: 5px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    color: black; \n"
"    font-size: 16px; \n"
"}\n"
""));

        horizontalLayout->addWidget(labelTargetX);

        labelTargetY = new QLabel(centralwidget);
        labelTargetY->setObjectName("labelTargetY");
        labelTargetY->setFont(font1);
        labelTargetY->setStyleSheet(QString::fromUtf8("QLabel {\n"
"    padding: 5px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    color: black; /* Kolor tekstu */\n"
"    font-size: 16px; /* Wi\304\231kszy rozmiar czcionki */\n"
"}\n"
""));

        horizontalLayout->addWidget(labelTargetY);

        lineEditTargetX = new QLineEdit(centralwidget);
        lineEditTargetX->setObjectName("lineEditTargetX");
        lineEditTargetX->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: 2px solid #4F6F52; \n"
"    border-radius: 5px; \n"
"    padding: 5px;\n"
"    background: white; /* Bia\305\202e t\305\202o */\n"
"    selection-background-color: #4CAF50;\n"
"    selection-color: white; \n"
"}\n"
"\n"
"QLineEdit:focus {\n"
"    border: 2px solid #FF0000;\n"
"}\n"
""));

        horizontalLayout->addWidget(lineEditTargetX);

        lineEditTargetY = new QLineEdit(centralwidget);
        lineEditTargetY->setObjectName("lineEditTargetY");
        lineEditTargetY->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: 2px solid #4F6F52; \n"
"    border-radius: 5px; \n"
"    padding: 5px; \n"
"    background: white; \n"
"    selection-background-color: #4F6F52; \n"
"    selection-color: white; \n"
"}\n"
"\n"
"QLineEdit:focus {\n"
"    border: 2px solid #FF0000\n"
"}\n"
""));

        horizontalLayout->addWidget(lineEditTargetY);

        buttonSendTarget = new QPushButton(centralwidget);
        buttonSendTarget->setObjectName("buttonSendTarget");
        buttonSendTarget->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    padding: 0px;\n"
"    margin: 0px;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"    background-color: white;\n"
"    background-position: center;\n"
"    background-repeat: no-repeat;\n"
"    background-size: contain;\n"
"}\n"
"\n"
"QPushButton#buttonSendTarget {\n"
"    background-color: white;\n"
"    border: 2px solid #4F6F52;\n"
"    border-radius: 5px;\n"
"}\n"
"\n"
"QPushButton#buttonSendTarget:pressed {\n"
"    background-color: #4F6F52;\n"
"    color: white; \n"
"}\n"
""));
        buttonSendTarget->setAutoDefault(false);

        horizontalLayout->addWidget(buttonSendTarget);

        horizontalLayout->setStretch(0, 1);
        horizontalLayout->setStretch(1, 1);
        horizontalLayout->setStretch(2, 1);
        horizontalLayout->setStretch(3, 1);
        horizontalLayout->setStretch(4, 1);

        verticalLayout_2->addLayout(horizontalLayout);


        verticalLayout_4->addLayout(verticalLayout_2);

        verticalLayout_4->setStretch(0, 4);
        verticalLayout_4->setStretch(1, 1);
        verticalLayout_4->setStretch(2, 1);

        main->addLayout(verticalLayout_4);


        horizontalLayout_6->addLayout(main);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 962, 24));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        ledIndicator->setText(QString());
        statusLabel->setText(QString());
        refreshButton->setText(QString());
        languageButton->setText(QString());
        buttonConnect->setText(QString());
        clearButton->setText(QString());
        pwn_text_L->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        pwmLProgressBar->setFormat(QCoreApplication::translate("MainWindow", "%p%", nullptr));
        pwn_text_R->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        speed_text_L->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        speedLProgressBar->setFormat(QCoreApplication::translate("MainWindow", "%p%", nullptr));
        speed_text_R->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        actual_position->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        labelPosY->setText(QCoreApplication::translate("MainWindow", "X", nullptr));
        labelPosX->setText(QCoreApplication::translate("MainWindow", "Y", nullptr));
        labelTheta->setText(QCoreApplication::translate("MainWindow", "Theta", nullptr));
        actualTarget->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        send_target->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        labelTargetX->setText(QString());
        labelTargetY->setText(QString());
        buttonSendTarget->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
