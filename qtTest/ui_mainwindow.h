/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Tue Nov 13 14:19:03 2012
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
    QAction *actionOpenPointCloud;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_2;
    MainWidget *widget;
    QGridLayout *gridLayout;
    QLabel *statusLabel;
    QPushButton *convertButton;
    QPushButton *toggleViewButton;
    QMenuBar *menuBar;
    QMenu *menuSerialPort;
    QMenu *menuCamera;
    QMenu *menuFile;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(619, 513);
        actionPort1 = new QAction(MainWindow);
        actionPort1->setObjectName(QString::fromUtf8("actionPort1"));
        actionOpenPointCloud = new QAction(MainWindow);
        actionOpenPointCloud->setObjectName(QString::fromUtf8("actionOpenPointCloud"));
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
        statusLabel = new QLabel(centralWidget);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));

        gridLayout->addWidget(statusLabel, 0, 0, 1, 1);

        convertButton = new QPushButton(centralWidget);
        convertButton->setObjectName(QString::fromUtf8("convertButton"));

        gridLayout->addWidget(convertButton, 4, 0, 1, 1);

        toggleViewButton = new QPushButton(centralWidget);
        toggleViewButton->setObjectName(QString::fromUtf8("toggleViewButton"));

        gridLayout->addWidget(toggleViewButton, 6, 0, 1, 1);


        verticalLayout_2->addLayout(gridLayout);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 619, 22));
        menuSerialPort = new QMenu(menuBar);
        menuSerialPort->setObjectName(QString::fromUtf8("menuSerialPort"));
        menuCamera = new QMenu(menuBar);
        menuCamera->setObjectName(QString::fromUtf8("menuCamera"));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuSerialPort->menuAction());
        menuBar->addAction(menuCamera->menuAction());
        menuSerialPort->addAction(actionPort1);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "FabScan", 0, QApplication::UnicodeUTF8));
        actionPort1->setText(QApplication::translate("MainWindow", "blablabla", 0, QApplication::UnicodeUTF8));
        actionOpenPointCloud->setText(QApplication::translate("MainWindow", "Open PointCloud...", 0, QApplication::UnicodeUTF8));
        statusLabel->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        convertButton->setText(QApplication::translate("MainWindow", "Compute Surface Mesh...", 0, QApplication::UnicodeUTF8));
        toggleViewButton->setText(QApplication::translate("MainWindow", "Toggle Point Cloud / Surface Mesh", 0, QApplication::UnicodeUTF8));
        menuSerialPort->setTitle(QApplication::translate("MainWindow", "SerialPort", 0, QApplication::UnicodeUTF8));
        menuCamera->setTitle(QApplication::translate("MainWindow", "Camera", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
