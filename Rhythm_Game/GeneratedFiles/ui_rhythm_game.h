/********************************************************************************
** Form generated from reading UI file 'rhythm_game.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RHYTHM_GAME_H
#define UI_RHYTHM_GAME_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Rhythm_GameClass
{
public:
    QWidget *centralWidget;
    QPushButton *start_button;
    QLabel *camera;
    QPushButton *exit_button;
    QPushButton *ranking_button;
    QLabel *label;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Rhythm_GameClass)
    {
        if (Rhythm_GameClass->objectName().isEmpty())
            Rhythm_GameClass->setObjectName(QStringLiteral("Rhythm_GameClass"));
        Rhythm_GameClass->resize(660, 500);
        centralWidget = new QWidget(Rhythm_GameClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        start_button = new QPushButton(centralWidget);
        start_button->setObjectName(QStringLiteral("start_button"));
        start_button->setGeometry(QRect(100, 500, 75, 23));
        camera = new QLabel(centralWidget);
        camera->setObjectName(QStringLiteral("camera"));
        camera->setGeometry(QRect(10, 10, 640, 480));
        exit_button = new QPushButton(centralWidget);
        exit_button->setObjectName(QStringLiteral("exit_button"));
        exit_button->setGeometry(QRect(250, 400, 75, 23));
        ranking_button = new QPushButton(centralWidget);
        ranking_button->setObjectName(QStringLiteral("ranking_button"));
        ranking_button->setGeometry(QRect(400, 400, 75, 23));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(150, 120, 291, 101));
        Rhythm_GameClass->setCentralWidget(centralWidget);
        camera->raise();
        label->raise();
        start_button->raise();
        exit_button->raise();
        ranking_button->raise();
        mainToolBar = new QToolBar(Rhythm_GameClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        Rhythm_GameClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Rhythm_GameClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Rhythm_GameClass->setStatusBar(statusBar);

        retranslateUi(Rhythm_GameClass);
        QObject::connect(start_button, SIGNAL(clicked()), Rhythm_GameClass, SLOT(start_button()));
        QObject::connect(exit_button, SIGNAL(clicked()), Rhythm_GameClass, SLOT(exit_button()));

        QMetaObject::connectSlotsByName(Rhythm_GameClass);
    } // setupUi

    void retranslateUi(QMainWindow *Rhythm_GameClass)
    {
        Rhythm_GameClass->setWindowTitle(QApplication::translate("Rhythm_GameClass", "Rhythm_Game", 0));
        start_button->setText(QApplication::translate("Rhythm_GameClass", "Start", 0));
        camera->setText(QString());
        exit_button->setText(QApplication::translate("Rhythm_GameClass", "Exit", 0));
        ranking_button->setText(QApplication::translate("Rhythm_GameClass", "Ranking", 0));
        label->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class Rhythm_GameClass: public Ui_Rhythm_GameClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RHYTHM_GAME_H
