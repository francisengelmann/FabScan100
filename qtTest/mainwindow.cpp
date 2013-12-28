#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "fsdialog.h"

#include <QBasicTimer>
#include <QDialogButtonBox>
#include <QFuture>
#include <QtCore>
//#include <QtConcurrentRun>
#include <QtConcurrent/QtConcurrentRun>
#include <QCamera>
#include <QSound>

#include <boost/bind.hpp>

#ifdef LINUX
#include <boost/filesystem.hpp>
#endif



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    hwTimer(new QBasicTimer),
    ui(new Ui::MainWindow)
{
    controller = FSController::getInstance();
    ui->setupUi(this);
    this->setupMenu();
    this->enumerateSerialPorts();
    this->enumerateWebCams();
    hwTimer->start(5000, this); //timer that checks periodically for attached hardware (camera, arduino)
    dialog = new FSDialog(this);
    controlPanel = new FSControlPanel(this);
    controller->mainwindow=this;
    controller->controlPanel=controlPanel;
    ui->widget->setStyleSheet("border: 1px solid black;");
    applyState(POINT_CLOUD);
    //resolution: Good
    controller->turntableStepSize = 16*controller->turntable->degreesPerStep;
    controller->yDpi = 1;
}

MainWindow::~MainWindow()
{
    controller->scanning=false;	// Terminate any scan in progress
    controller->laser->turnOff();	// Make sure laser is off!
    controller->destroy();	// We'll find what's hogging the .exe the hard way...
	cv::destroyAllWindows();
	controlPanel->~FSControlPanel();
	delete ui;
	dialog->~FSDialog();
	//qextSerialEnumerator->~QextSerialEnumerator();
	hwTimer->~QBasicTimer();
	qApp->quit();
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

    QAction* exportSTLAction = new QAction("Export .STL...", this);
    connect(exportSTLAction,SIGNAL(triggered()),this, SLOT(exportSTL()));
    ui->menuFile->addAction(exportSTLAction);

    QAction* readConfiguartion = new QAction("Read Configuration", this);
    connect(readConfiguartion,SIGNAL(triggered()),this, SLOT(readConfiguration()));
    ui->menuFile->addAction(readConfiguartion);

    QAction* showControlPanelAction = new QAction("Control Panel...", this);
    showControlPanelAction->setShortcuts(QKeySequence::Preferences);
    connect(showControlPanelAction,SIGNAL(triggered()),this, SLOT(showControlPanel()));
    ui->menuFile->addAction(showControlPanelAction);
}

void MainWindow::setupCamWindows()
{
    //Below is the added code to display the two cv camera feeds, which attach to the side of MainWindow.ui
    cv::namedWindow(WINDOW_EXTRACTED_FRAME);
    cv::namedWindow(WINDOW_LASER_FRAME);

#ifdef WINDOWS
    HWND hWnd0 = (HWND)cvGetWindowHandle("FabScan");
    HWND hRawWnd0 = ::GetParent(hWnd0);
    if (hRawWnd0 != NULL) {
        BOOL bRet = ::SetWindowPos(hRawWnd0, HWND_TOPMOST, 100, 100, 0, 0, SWP_NOSIZE );
        assert(bRet);
    }

    HWND hWnd = (HWND)cvGetWindowHandle(WINDOW_EXTRACTED_FRAME);
    HWND hRawWnd = ::GetParent(hWnd);
    if (hRawWnd != NULL) {
        BOOL bRet = ::SetWindowPos(hRawWnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOSIZE |SWP_NOMOVE);
        assert(bRet);
    }

    HWND hWnd2 = (HWND)cvGetWindowHandle(WINDOW_LASER_FRAME);
    HWND hRawWnd2 = ::GetParent(hWnd2);
    if (hRawWnd2 != NULL) {
        BOOL bRet = ::SetWindowPos(hRawWnd2, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOSIZE |SWP_NOMOVE);
        assert(bRet);
    }
#endif

    cv::namedWindow(WINDOW_EXTRACTED_FRAME);
    cv::namedWindow(WINDOW_LASER_FRAME);

    cv::resizeWindow(WINDOW_EXTRACTED_FRAME, controller->mainwindow->height()*3/4,controller->mainwindow->height()/2-19);
    cv::resizeWindow(WINDOW_LASER_FRAME, controller->mainwindow->height()*3/4,controller->mainwindow->height()/2-19);
    cv::moveWindow(WINDOW_EXTRACTED_FRAME, controller->mainwindow->x() +controller->mainwindow->width()+15,controller->mainwindow->y());
    cv::moveWindow(WINDOW_LASER_FRAME, controller->mainwindow->x() +controller->mainwindow->width()+15,controller->mainwindow->y()+controller->mainwindow->height()/2+19);
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

void MainWindow::exportSTL()
{
    if(controller->model->pointCloud->empty()){
        this->showDialog("PointCloud is empty! Perform a scan, or open a pointcloud.");
        return;
    }
    QFileDialog d(this, "Save File","","STL (*.stl)");
    d.setAcceptMode(QFileDialog::AcceptSave);
    if(d.exec()){

        QString fileName = d.selectedFiles()[0];
        if(fileName.isEmpty() ) return;
        qDebug() << fileName;

        if(!controller->meshComputed){
            qDebug() << "Computing mesh...";
            this->showDialog("Will now compute surface mesh, this may take a while...");
            controller->computeSurfaceMesh();
            controller->meshComputed = true;
        }
        qDebug() << "Done computing surface mesh, now stl export...";
        controller->model->saveToSTLFile(fileName.toStdString());
        this->showDialog("STL export done!");

    }
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

    //Added for the OpenCV webcam windows. They update each timer period

    cv::resizeWindow(WINDOW_EXTRACTED_FRAME, controller->mainwindow->height()*3/4,controller->mainwindow->height()/2-19);
    cv::resizeWindow(WINDOW_LASER_FRAME, controller->mainwindow->height()*3/4,controller->mainwindow->height()/2-19);
    cv::moveWindow(WINDOW_EXTRACTED_FRAME, controller->mainwindow->x() +controller->mainwindow->width()+15,controller->mainwindow->y());
    cv::moveWindow(WINDOW_LASER_FRAME, controller->mainwindow->x() +controller->mainwindow->width()+15,controller->mainwindow->y()+controller->mainwindow->height()/2+19);


    //result = controller->isArduinoAlive(); //See if Arduino still responding properly
}

//===========================================
// Menu Methods
//===========================================

void MainWindow::onSelectSerialPort()
{
    QAction* action=qobject_cast<QAction*>(sender());
    if(!action) return;

    if(controller->serial->serialPortPath->compare(action->iconText()) == 0)
    {
        // the clicked port is the port, already opened by the controller
        controller->serial->disconnectFromSerialPort();
        action->setChecked(false);
    }
    else
    {
        // this port is not opened by the controller

        // If the controller is already connected to another serial port
        if(controller->serial->serialPortPath->size())	//i.e. is the path name more than zero characters long?
        {	//Yes, disconnect it
            controller->serial->disconnectFromSerialPort();
        }

        //set new path
        controller->serial->serialPortPath->clear();
        controller->serial->serialPortPath->append(action->iconText());
        bool result = controller->serial->connectToSerialPort();
        action->setChecked(result);
        ui->Serial_check->setChecked(result);
        qDebug() << "Selected serial port: " << controller->serial->serialPortPath << "\tisArduinoAlive? " << result;
    }
}



void MainWindow::onSelectWebCam()
{
    QAction* action=qobject_cast<QAction*>(sender());
    if(!action) return;
    controller->webcam->info.portName=action->iconText(); //eigentlich doppelt gemoppelt, das hier kann weg muss jedoch gekukt werden
    controller->webcam->setCamera(action->data().toByteArray());

    if(controller->webcam->info.portName.size())
    {
        action->setChecked(true);
        ui->Camera_check->setChecked(true);
        this->setupCamWindows();
    }
    // TODO: Disconnect camera?
}

void MainWindow::openPointCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this, "Open File","","Files (*.pcd) ;; PLY (*.ply)");
    if(fileName.isEmpty() ) return;
    if(fileName.endsWith(".pcd", Qt::CaseInsensitive) ){
        controller->model->loadPointCloudFromPCD(fileName.toStdString());
    }else if(fileName.endsWith(".ply", Qt::CaseInsensitive) ){
        controller->model->loadPointCloudFromPLY(fileName.toStdString());
    }
    ui->widget->drawState = 0;
    ui->widget->updateGL();
    applyState(POINT_CLOUD);
    controller->meshComputed=false;
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
            controller->model->savePointCloudAsPCD(fileName.toStdString());
        }else if(fileName.endsWith(".ply", Qt::CaseInsensitive) ){
            qDebug() << "Save as ply file.";
            controller->model->savePointCloudAsPLY(fileName.toStdString());
        }
    }

    ui->widget->drawState = 0;
    ui->widget->updateGL();
}

void MainWindow::newPointCloud()
{
    controller->model->pointCloud->clear();
    controller->model->surfaceMesh.polygons.clear();
    ui->widget->updateGL();
    applyState(POINT_CLOUD);
    controller->meshComputed=false;
}

void MainWindow::readConfiguration()
{
    if(FSController::config->readConfiguration()){
        this->showDialog("Successfully read configuration file!");
    }else{
        this->showDialog("Configuration file not found or corrupt!");
    }
}

void MainWindow::enumerateSerialPorts()
{
    //bool status;

    QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();	// Get list of serial ports available
    ui->menuSerialPort->clear();	//Clear list in FabScan menu

    foreach (QextPortInfo info, ports) {
        if(!info.portName.isEmpty()){	//If there is a port name...
            QAction* ac = new QAction(info.portName, this); //... add this to the menu list...
            ac->setCheckable(true);	//...which the user can select
            connect(ac,SIGNAL(triggered()),this, SLOT(onSelectSerialPort())); //Really, this only needs to be done once, not for each port
            if(controller->serial->serialPortPath->compare(info.portName)==0){
                ac->setChecked(true);	//If ?
            }
            ui->menuSerialPort->addAction(ac);
        }
    }
}

void MainWindow::enumerateWebCams()
{
    QList<FSWebCamInfo> cameras = controller->webcam->getCameras();
    ui->menuCamera->clear();

    if(cameras.size()==0){
       QAction* a = new QAction("No camera found.", this);
       a->setEnabled(false);
       ui->menuCamera->clear();
       ui->menuCamera->addAction(a);
       return;
    }

    foreach(const FSWebCamInfo &cameraInfo, cameras)
    {
        QAction *videoDeviceAction = new QAction(cameraInfo.friendlyName, this);
        videoDeviceAction->setCheckable(true);
        videoDeviceAction->setData(cameraInfo.deviceName);
        connect(videoDeviceAction,SIGNAL(triggered()),this, SLOT(onSelectWebCam()));

        ui->menuCamera->addAction(videoDeviceAction);
    }
}

void MainWindow::on_scanButton_clicked()
{
    if(controller->scanning)
    {
        // if already scanning: cancel
        applyState(POINT_CLOUD);
        this->ui->scanButton->setText("Start Scan");
        controller->laser->turnOff();	// Make sure laser is off!
        controller->scanning = false;
    }
    else
    {
        // if not scanning: start
        applyState(SCANNING);
        controller->scan();
    }

}

void MainWindow::doneScanning()
{
    QSound::play("done.wav");
    this->ui->scanButton->setText("Start Scan");
    controller->laser->disable();
    controller->turntable->disable();
    applyState(POINT_CLOUD);
}

void MainWindow::redraw()
{
    //ui->widget->drawState = 0;
    ui->widget->updateGL();
}

void MainWindow::applyState(FSState s)
{
    state = s;
    switch(state){
    case SCANNING:
        this->ui->widget->drawState=0;
        this->ui->scanButton->setText("Stop Scan");
        break;
    case POINT_CLOUD:
        this->ui->scanButton->setText("Start Scan");
        //the following lines are uncommented since we do not support showing the mesh anymore but just compute and save it
        /*if(controller->meshComputed){
            this->ui->toggleViewButton->setText("Show SurfaceMesh");
        }else{
            this->ui->toggleViewButton->setText("Compute SurfaceMesh");
        }*/
        break;
    case SURFACE_MESH:
        this->ui->scanButton->setText("Start Scan");
        break;
    }
}

void MainWindow::on_resolutionComboBox_currentIndexChanged(const QString &arg1)
{
    if(arg1.compare("Best")==0){
        //laserStepSize = 2*laser->degreesPerStep;
        controller->turntableStepSize = controller->turntable->degreesPerStep;
        controller->yDpi = 1;
    }
    if(arg1.compare("Good")==0){
        controller->turntableStepSize = 16*controller->turntable->degreesPerStep;
        controller->yDpi = 1;
    }
    if(arg1.compare("Normal")==0){
        controller->turntableStepSize = 2*16*controller->turntable->degreesPerStep;
        controller->yDpi = 5;
    }
    if(arg1.compare("Poor")==0){
        controller->turntableStepSize = 10*16*controller->turntable->degreesPerStep;
        controller->yDpi = 10;
    }
}
