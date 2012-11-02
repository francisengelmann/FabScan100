/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Nov 2 17:24:22 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "mainwidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionPort1;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    MainWidget *widget;
    QGridLayout *gridLayout;
    QLabel *label;
    QPushButton *convertButton;
    QPushButton *pushButton;
    QPushButton *myButton;
    QPushButton *toggleViewButton;
    QMenuBar *menuBar;
    QMenu *menuSerialPort;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(619, 513);
        actionPort1 = new QAction(MainWindow);
        actionPort1->setObjectName(QString::fromUtf8("actionPort1"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout_2 = new QVBoxLayout(centralWidget);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        widget = new MainWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(widget);

        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        convertButton = new QPushButton(centralWidget);
        convertButton->setObjectName(QString::fromUtf8("convertButton"));

        gridLayout->addWidget(convertButton, 2, 0, 1, 1);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 4, 0, 1, 1);

        myButton = new QPushButton(centralWidget);
        myButton->setObjectName(QString::fromUtf8("myButton"));

        gridLayout->addWidget(myButton, 1, 0, 1, 1);

        toggleViewButton = new QPushButton(centralWidget);
        toggleViewButton->setObjectName(QString::fromUtf8("toggleViewButton"));

        gridLayout->addWidget(toggleViewButton, 3, 0, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 619, 22));
        menuSerialPort = new QMenu(menuBar);
        menuSerialPort->setObjectName(QString::fromUtf8("menuSerialPort"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuSerialPort->menuAction());
        menuSerialPort->addAction(actionPort1);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "FabScan", 0, QApplication::UnicodeUTF8));
        actionPort1->setText(QApplication::translate("MainWindow", "blablabla", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        convertButton->setText(QApplication::translate("MainWindow", "Compute Surface Mesh...", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Open PointCloud...", 0, QApplication::UnicodeUTF8));
        myButton->setText(QApplication::translate("MainWindow", "Start Scan", 0, QApplication::UnicodeUTF8));
        toggleViewButton->setText(QApplication::translate("MainWindow", "Toggle Point Cloud / Surface Mesh", 0, QApplication::UnicodeUTF8));
        menuSerialPort->setTitle(QApplication::translate("MainWindow", "SerialPort", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
