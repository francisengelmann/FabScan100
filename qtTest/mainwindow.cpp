#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fscontroller.h"
#include "fsdialog.h"

#include <QBasicTimer>
#include <QDialogButtonBox>
#include <QFuture>
#include <QtCore>
#include <QtConcurrentRun>

#include <boost/bind.hpp>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    hwTimer(new QBasicTimer),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setupMenu();
    this->enumerateSerialPorts();
    this->enumerateWebCams();
    hwTimer->start(5000, this);
    ui->statusLabel->setText("Not connected to FabScan.");
    dialog = new FSDialog(this);
    controlPanel = new FSControlPanel(this);
    FSController::getInstance()->mainwindow=this;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupMenu()
{
    QAction* newPointCloudAction = new QAction("New", this);
    newPointCloudAction->setShortcuts(QKeySequence::New);
    connect(newPointCloudAction,SIGNAL(triggered()),this, SLOT(newPointCloud()));
    ui->menuFile->addAction(newPointCloudAction);

    QAction* openPointCloudAction = new QAction("Open PointCloud...", this);
    openPointCloudAction->setShortcuts(QKeySequence::Open);
    connect(openPointCloudAction,SIGNAL(triggered()),this, SLOT(openPointCloud()));
    ui->menuFile->addAction(openPointCloudAction);

    QAction* savePointCloudAction = new QAction("Save PointCloud...", this);
    savePointCloudAction->setShortcuts(QKeySequence::Save);
    connect(savePointCloudAction,SIGNAL(triggered()),this, SLOT(savePointCloud()));
    ui->menuFile->addAction(savePointCloudAction);

    QAction* showControlPanelAction = new QAction("Control Panel...", this);
    showControlPanelAction->setShortcuts(QKeySequence::Preferences);
    connect(showControlPanelAction,SIGNAL(triggered()),this, SLOT(showControlPanel()));
    ui->menuFile->addAction(showControlPanelAction);
}

void MainWindow::showDialog(QString dialogText)
{
    dialog->setStandardButtons(QDialogButtonBox::Ok);
    dialog->setText(dialogText);
    dialog->show();
    dialog->raise();
    dialog->activateWindow();
}

//===========================================
// Action Methods
//===========================================

void MainWindow::on_convertButton_clicked()
{
    FSController::getInstance()->computeSurfaceMesh();
    ui->widget->drawState = 1; //display surface mesh
    ui->widget->updateGL();
}

void MainWindow::on_toggleViewButton_clicked()
{
    char currentDrawState = ui->widget->drawState;
    ui->widget->drawState = 1-currentDrawState;
    ui->widget->updateGL();
}

void MainWindow::showControlPanel()
{
    controlPanel->show();
    controlPanel->raise();
    controlPanel->activateWindow();
}

void MainWindow::timerEvent(QTimerEvent *e)
{
    Q_UNUSED(e);
    //this->enumerateSerialPorts();
    //this->enumerateWebCams();
}

//===========================================
// Menu Methods
//===========================================

void MainWindow::onSelectSerialPort()
{
    QAction* action=qobject_cast<QAction*>(sender());
    if(!action) return;
    //set new path
    FSController::getInstance()->serial->serialPortPath->clear();
    FSController::getInstance()->serial->serialPortPath->append(action->iconText());
    this->enumerateSerialPorts();
    FSController::getInstance()->serial->connectToSerialPort();
    ui->statusLabel->setText(QString("Now connected to").append(action->iconText()));
}

void MainWindow::onSelectWebCam()
{
    QAction* action=qobject_cast<QAction*>(sender());
    if(!action) return;
    FSController::getInstance()->webcam->info.portName=action->iconText();
    this->enumerateWebCams();
}

void MainWindow::openPointCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File","","Files (*.pcd)");
    FSController::getInstance()->model->loadPointCloud(fileName.toStdString());
    ui->widget->drawState = 0;
    ui->widget->updateGL();
}

void MainWindow::savePointCloud()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save File","","Files (*.pcd)");
    FSController::getInstance()->model->savePointCloud(fileName.toStdString());
    ui->widget->drawState = 0;
    ui->widget->updateGL();
}

void MainWindow::newPointCloud()
{
    FSController::getInstance()->model->pointCloud->clear();
    FSController::getInstance()->model->triangles.polygons.clear();
    ui->widget->updateGL();
}

void MainWindow::enumerateSerialPorts()
{
    QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();
    ui->menuSerialPort->clear();

    foreach (QextPortInfo info, ports) {
        if(!info.portName.isEmpty() && !info.portName.startsWith("ttyS")){
            QAction* ac = new QAction(info.portName, this);
            ac->setCheckable(true);
            connect(ac,SIGNAL(triggered()),this, SLOT(onSelectSerialPort()));
            if(FSController::getInstance()->serial->serialPortPath->compare(info.portName)==0){
                ac->setChecked(true);
            }
            ui->menuSerialPort->addAction(ac);
        }
        //qDebug() << "port name:"       << info.portName;
        //qDebug() << "friendly name:"   << info.friendName;
        //qDebug() << "physical name:"   << info.physName;
        //qDebug() << "enumerator name:" << info.enumName;
        //qDebug() << "vendor ID:"       << info.vendorID;
        //qDebug() << "product ID:"      << info.productID;
        //qDebug() << "===================================";
    }
}

void MainWindow::enumerateWebCams()
{
    QList<FSWebCamInfo> ports = FSWebCam::enumerate();
    if(ports.size()==0){
       QAction* a = new QAction("No camera found", this);
       a->setEnabled(false);
       ui->menuCamera->clear();
       ui->menuCamera->addAction(a);
       return;
    }

    ui->menuCamera->clear();
    foreach (FSWebCamInfo cam, ports) {
        if(!cam.portName.isEmpty()){
            QAction* ac = new QAction(cam.portName, this);
            ac->setCheckable(true);
            connect(ac,SIGNAL(triggered()),this, SLOT(onSelectWebCam()));
            if(FSController::getInstance()->webcam->info.portName.compare(cam.portName)==0){
                ac->setChecked(true);
            }
            ui->menuCamera->addAction(ac);
        }
    }
}

void MainWindow::on_scanButton_clicked()
{
    //QFuture<void> future = QtConcurrent::run(FSController::getInstance(), &FSController::scanThread);
    FSController::getInstance()->scanThread();
}

void MainWindow::redraw()
{
    ui->widget->drawState = 0;
    ui->widget->updateGL();
}
