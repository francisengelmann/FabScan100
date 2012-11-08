#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "fscontroller.h"

#include <QBasicTimer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    hwTimer(new QBasicTimer),
    ui(new Ui::MainWindow)
    //serialPortPath(new QString)
{
    ui->setupUi(this);
    this->setupMenu();
    this->enumerateSerialPorts();
    hwTimer->start(5000, this);
    ui->statusLabel->setText("Not connected to FabScan.");
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
}

//===========================================
// Action Methods
//===========================================

void MainWindow::on_myButton_clicked()
{
    return;
    //image= cv::imread("cube.png");
    //cv::namedWindow("Original Image");
    //cv::imshow("Original Image", image);
    ui->statusLabel->setText("Press a button to close...");
    qDebug("Hello World!");

    // Open the video file
    cv::VideoCapture capture(-1);
    // check if video successfully opened
    if (!capture.isOpened()) return;
    // Get the frame rate
          double rate= capture.get(CV_CAP_PROP_FPS);
          bool stop(false);
          cv::Mat frame; // current video frame
          cv::namedWindow("Extracted Frame");
          // Delay between each frame in ms
          // corresponds to video frame rate
          int delay= 1000/rate;
          // for all frames in video
          while (!stop) {
             // read next frame if any
             if (!capture.read(frame))
                break;
             cv::imshow("Extracted Frame",frame);
             // introduce a delay
             // or press key to stop
             if (cv::waitKey(1)>=0)
            stop = true;
    }
          // Close the video file.
          // Not required since called by destructor
          capture.release();
          cvDestroyWindow("Extracted Frame");
          ui->statusLabel->setText("Idling...");
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
    this->enumerateSerialPorts();
    this->enumerateWebCams();
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
    //FSController::getInstance()->serial->serialPortPath->append(action->iconText());
    this->enumerateWebCams();
    //FSController::getInstance()->serial->connectToSerialPort();
    ui->statusLabel->setText(QString("Now connected to").append(action->iconText()));
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
        if(!info.portName.isEmpty()){
        //ui->menuSerialPort->addAction(info.portName,)
            QAction* ac = new QAction(info.portName, this);
            ac->setCheckable(true);
            connect(ac,SIGNAL(triggered()),this, SLOT(onSelectSerialPort()));
            if(FSController::getInstance()->serial->serialPortPath->compare(info.portName)==0){
                ac->setChecked(true);
            }
            //ui->menuSerialPort->addAction(info.portName, this, SLOT(selectSerialPort()));
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
    ui->menuCamera->clear();

    foreach (FSWebCamInfo cam, ports) {
        if(!cam.portName.isEmpty()){
        //ui->menuSerialPort->addAction(info.portName,)
            QAction* ac = new QAction(cam.portName, this);
            ac->setCheckable(true);
            connect(ac,SIGNAL(triggered()),this, SLOT(onSelectWebCam()));
            //if(FSController::getInstance()->serial->serialPortPath->compare(info.portName)==0){
                //ac->setChecked(true);
            //}
            //ui->menuSerialPort->addAction(info.portName, this, SLOT(selectSerialPort()));
            ui->menuCamera->addAction(ac);
        }
    }

    if(ports.size()==0){
       QAction* a = new QAction("No camera found", this);
       a->setEnabled(false);
       ui->menuCamera->addAction(a);
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
