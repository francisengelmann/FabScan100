#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fscontroller.h"
#include "fsdialog.h"

#include <QBasicTimer>
#include <opencv2/imgproc/imgproc.hpp>

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

    QAction* showControlPanelAction = new QAction("Control Panel...", this);
    showControlPanelAction->setShortcuts(QKeySequence::Open);
    connect(showControlPanelAction,SIGNAL(triggered()),this, SLOT(openPointCloud()));
    ui->menuFile->addAction(showControlPanelAction);
}

void MainWindow::showDialog(QString dialogText)
{
    dialog->setText(dialogText);
    dialog->show();
    dialog->raise();
    dialog->activateWindow();
}

//===========================================
// Action Methods
//===========================================

void MainWindow::on_myButton_clicked()
{
    if(FSController::getInstance()->webcam->info.portName.isEmpty()){
        showDialog("No webcam selected!");
        return;
    }

    ui->statusLabel->setText("Press a button to close...");
    cv::Mat frame;
    frame = FSController::getInstance()->webcam->getFrame();
    cv::resize(frame,frame,cv::Size(400,300));
    cv::imshow("Extracted Frame",frame);
    cv::waitKey(0);
    cvDestroyWindow("Extracted Frame");
}

void MainWindow::on_convertButton_clicked()
{
    if(FSController::getInstance()->model->pointCloud->empty()){
        return;
    }
    qDebug("converting...");
    FSController::getInstance()->model->convertPointCloudToSurfaceMesh();
    FSController::getInstance()->geometries->setSurfaceMeshTo(
                FSController::getInstance()->model->triangles,
                FSController::getInstance()->model->pointCloud);
    ui->widget->drawState = 1; //display surface mesh
    ui->widget->updateGL();
}

void MainWindow::on_toggleViewButton_clicked()
{
    char currentDrawState = ui->widget->drawState;
    ui->widget->drawState = 1-currentDrawState;
    ui->widget->updateGL();
}

void MainWindow::on_pingButton_clicked()
{
    FSController::getInstance()->serial->writeChar(200);
    /*if(serialPortPath->isEmpty())
        return;
    qDebug("button pressed");
    if(!serialPort->isOpen() )
        return;

    if(serialPort->isWritable() ){
        qDebug("is writable");
        const char c = 255;
        serialPort->write(&c);
    }else{
        qDebug("is not writable");
    }*/
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

/*bool MainWindow::connectToSerialPort() //outdated
{
    this->serialPort = new QextSerialPort(*serialPortPath, QextSerialPort::EventDriven);
    serialPort->setBaudRate(BAUD9600);
    serialPort->setFlowControl(FLOW_OFF);
    serialPort->setParity(PAR_NONE);
    serialPort->setDataBits(DATA_8);
    serialPort->setStopBits(STOP_2);

    if (serialPort->open(QIODevice::ReadWrite) == true) {
        connect(serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));
        connect(serialPort, SIGNAL(dsrChanged(bool)), this, SLOT(onDsrChanged(bool)));
        if (!(serialPort->lineStatus() & LS_DSR)){
            qDebug() << "warning: device is not turned on";
            return false;
        }
        qDebug() << "listening for data on" << serialPort->portName();
        return true;
    }else{
        qDebug() << "device failed to open:" << serialPort->errorString();
        return true;
    }
}*/

void MainWindow::on_laserOnButton_clicked()
{
    FSController::getInstance()->serial->writeChar(201);
}
