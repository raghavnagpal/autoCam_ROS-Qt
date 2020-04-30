/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionMenu;
    QWidget *centralwidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QPushButton *SetPoint_SaveViewButton;
    QPushButton *SetPoint_HomeButton;
    QPushButton *Follow_PlayButton;
    QPushButton *SetPoint_TopViewButton;
    QPushButton *Follow_ResetButton;
    QPushButton *SetPoint_SideViewButton;
    QPushButton *Manual_SwitchButton;
    QGraphicsView *graphicsView;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(960, 780);
        MainWindow->setMinimumSize(QSize(960, 780));
        MainWindow->setMaximumSize(QSize(960, 780));
        MainWindow->setStyleSheet(QString::fromUtf8("background-color: rgb(40,40,40);\n"
"color: lightgrey;\n"
"QPushButton{\n"
"border: 1px solid red;\n"
"    border-radius: 2px;\n"
"    background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1,\n"
"                                      stop: 0 rgb(72,72,72), stop: 1 #dadbde);\n"
"}\n"
"statusBar{\n"
"background-color: grey;\n"
"}\n"
"\n"
""));
        actionMenu = new QAction(MainWindow);
        actionMenu->setObjectName(QString::fromUtf8("actionMenu"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayoutWidget = new QWidget(centralwidget);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(30, 20, 913, 41));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        SetPoint_SaveViewButton = new QPushButton(gridLayoutWidget);
        SetPoint_SaveViewButton->setObjectName(QString::fromUtf8("SetPoint_SaveViewButton"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SetPoint_SaveViewButton->sizePolicy().hasHeightForWidth());
        SetPoint_SaveViewButton->setSizePolicy(sizePolicy);
        SetPoint_SaveViewButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(SetPoint_SaveViewButton, 0, 3, 1, 1);

        SetPoint_HomeButton = new QPushButton(gridLayoutWidget);
        SetPoint_HomeButton->setObjectName(QString::fromUtf8("SetPoint_HomeButton"));
        sizePolicy.setHeightForWidth(SetPoint_HomeButton->sizePolicy().hasHeightForWidth());
        SetPoint_HomeButton->setSizePolicy(sizePolicy);
        SetPoint_HomeButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(SetPoint_HomeButton, 0, 0, 1, 1);

        Follow_PlayButton = new QPushButton(gridLayoutWidget);
        Follow_PlayButton->setObjectName(QString::fromUtf8("Follow_PlayButton"));
        sizePolicy.setHeightForWidth(Follow_PlayButton->sizePolicy().hasHeightForWidth());
        Follow_PlayButton->setSizePolicy(sizePolicy);
        Follow_PlayButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(Follow_PlayButton, 0, 4, 1, 1);

        SetPoint_TopViewButton = new QPushButton(gridLayoutWidget);
        SetPoint_TopViewButton->setObjectName(QString::fromUtf8("SetPoint_TopViewButton"));
        sizePolicy.setHeightForWidth(SetPoint_TopViewButton->sizePolicy().hasHeightForWidth());
        SetPoint_TopViewButton->setSizePolicy(sizePolicy);
        SetPoint_TopViewButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(SetPoint_TopViewButton, 0, 1, 1, 1);

        Follow_ResetButton = new QPushButton(gridLayoutWidget);
        Follow_ResetButton->setObjectName(QString::fromUtf8("Follow_ResetButton"));
        sizePolicy.setHeightForWidth(Follow_ResetButton->sizePolicy().hasHeightForWidth());
        Follow_ResetButton->setSizePolicy(sizePolicy);
        Follow_ResetButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(Follow_ResetButton, 0, 5, 1, 1);

        SetPoint_SideViewButton = new QPushButton(gridLayoutWidget);
        SetPoint_SideViewButton->setObjectName(QString::fromUtf8("SetPoint_SideViewButton"));
        sizePolicy.setHeightForWidth(SetPoint_SideViewButton->sizePolicy().hasHeightForWidth());
        SetPoint_SideViewButton->setSizePolicy(sizePolicy);
        SetPoint_SideViewButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(SetPoint_SideViewButton, 0, 2, 1, 1);

        Manual_SwitchButton = new QPushButton(gridLayoutWidget);
        Manual_SwitchButton->setObjectName(QString::fromUtf8("Manual_SwitchButton"));
        sizePolicy.setHeightForWidth(Manual_SwitchButton->sizePolicy().hasHeightForWidth());
        Manual_SwitchButton->setSizePolicy(sizePolicy);
        Manual_SwitchButton->setMinimumSize(QSize(125, 25));

        gridLayout->addWidget(Manual_SwitchButton, 0, 6, 1, 1);

        graphicsView = new QGraphicsView(centralwidget);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(30, 80, 881, 651));
        MainWindow->setCentralWidget(centralwidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        statusBar->setLayoutDirection(Qt::LeftToRight);
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionMenu->setText(QApplication::translate("MainWindow", "Menu", 0, QApplication::UnicodeUTF8));
        SetPoint_SaveViewButton->setText(QApplication::translate("MainWindow", "Save/Edit View", 0, QApplication::UnicodeUTF8));
        SetPoint_HomeButton->setText(QApplication::translate("MainWindow", "Home", 0, QApplication::UnicodeUTF8));
        Follow_PlayButton->setText(QApplication::translate("MainWindow", "Play", 0, QApplication::UnicodeUTF8));
        SetPoint_TopViewButton->setText(QApplication::translate("MainWindow", "Top View", 0, QApplication::UnicodeUTF8));
        Follow_ResetButton->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
        SetPoint_SideViewButton->setText(QApplication::translate("MainWindow", "Side View", 0, QApplication::UnicodeUTF8));
        Manual_SwitchButton->setText(QApplication::translate("MainWindow", "Switch", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
