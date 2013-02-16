#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fscontroller.h"
#include "fsdialog.h"

#include <QBasicTimer>
#include <QDialogButtonBox>
#include <QFuture>
#include <QtCore>
//#include <QtConcurrentRun>
#include <QtConcurrent/QtConcurrentRun>
#include <QCamera>

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
    dialog = new FSDialog(this);
    controlPanel = new FSControlPanel(this);
    FSController::getInstance()->mainwindow=this;
    FSController::getInstance()->controlPanel=controlPanel;
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
}

void MainWindow::onSelectWebCam()
{
    QAction* action=qobject_cast<QAction*>(sender());
    if(!action) return;
    FSController::getInstance()->webcam->info.portName=action->iconText(); //eigentlich doppelt gemoppelt, das hier kann weg muss jedoch gekukt werden
    FSController::getInstance()->webcam->setCamera(action->data().toByteArray());
    this->enumerateWebCams();
}

void MainWindow::openPointCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File","","Files (*.pcd)");
    if(fileName.isEmpty() ) return;
    FSController::getInstance()->model->loadPointCloud(fileName.toStdString());
    ui->widget->drawState = 0;
    ui->widget->updateGL();
}

void MainWindow::savePointCloud()
{
    QFileDialog d(this, "Save File","","PCD (*.pcd) ;; PLY (*.ply)");
    d.setAcceptMode(QFileDialog::AcceptSave);
    if(d.exec()){
        QString fileName = d.selectedFiles()[0];
        //fileName.append(d.selectedNameFilter());
        if(fileName.isEmpty() ) return;
        qDebug() << fileName;
        if(fileName.endsWith(".pcd", Qt::CaseInsensitive) ){
            qDebug() << "Save as pcd file.";
            FSController::getInstance()->model->savePointCloudAsPCD(fileName.toStdString());
        }else if(fileName.endsWith(".ply", Qt::CaseInsensitive) ){
            qDebug() << "Save as ply file.";
            FSController::getInstance()->model->savePointCloudAsPLY(fileName.toStdString());
        }
    }

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
    if(QCamera::availableDevices().size()==0){
       QAction* a = new QAction("No camera found.", this);
       a->setEnabled(false);
       ui->menuCamera->clear();
       ui->menuCamera->addAction(a);
       return;
    }

    QByteArray cameraDevice;
    QCamera* camera;
    ui->menuCamera->clear();
    foreach(const QByteArray &deviceName, QCamera::availableDevices()) {
        QString description = camera->deviceDescription(deviceName);
        QAction *videoDeviceAction = new QAction(description, this);
        videoDeviceAction->setCheckable(true);
        videoDeviceAction->setData(QVariant(deviceName));
        connect(videoDeviceAction,SIGNAL(triggered()),this, SLOT(onSelectWebCam()));
        if (FSController::getInstance()->webcam->info.portName.compare(description)==0) {
            //cameraDevice = deviceName;
            videoDeviceAction->setChecked(true);
        }
        ui->menuCamera->addAction(videoDeviceAction);
    }
}

void MainWindow::on_scanButton_clicked()
{
    //QFuture<void> future = QtConcurrent::run(FSController::getInstance(), &FSController::scanThread);
    bool s = FSController::getInstance()->scanning;
    if (s==false){
        FSController::getInstance()->scanThread();
    }else{
        FSController::getInstance()->scanning = false;
    }
}

void MainWindow::redraw()
{
    ui->widget->drawState = 0;
    ui->widget->updateGL();
}
