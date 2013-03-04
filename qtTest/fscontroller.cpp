#include "fscontroller.h"
#include "fsdialog.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <boost/lexical_cast.hpp>

FSController* FSController::singleton=0;

FSController::FSController()
{
    //Software
    geometries = new GeometryEngine();
    model = new FSModel();
    //Harware
    serial = new FSSerial();
    webcam = new FSWebCam();
    turntable = new FSTurntable();
    laser = new FSLaser();
    vision = new FSVision();
    scanning = false;
    threshold = 40;
    //all in degrees;
    laserSwipeMin = 30; //18
    laserSwipeMax = 45; //50
    meshComputed = false;
    //old = true;
}

FSController* FSController::getInstance()
{
    if (singleton == 0){
        singleton = new FSController();
    }
    return singleton;
}

void FSController::destroy()
{
    if (singleton != 0) {
        delete singleton;
        singleton = 0;
    }
}

void FSController::fetchFrame()
{
    if(webcam->info.portName.isEmpty()){
        mainwindow->showDialog("No webcam selected!");
        return;
    }

    cv::Mat frame;
    frame = FSController::getInstance()->webcam->getFrame();
    //cv::imshow("Extracted Frame",frame);
    //cv::waitKey(0);
    cv::resize(frame,frame,cv::Size(1280,960));
    cv::Mat result = vision->drawHelperLinesToFrame(frame);
    cv::resize(result,result,cv::Size(800,600));
    cv::imshow("Extracted Frame",result);
    cv::waitKey(0);
    cvDestroyWindow("Extracted Frame");
}

void FSController::hideFrame()
{
    cvDestroyWindow("Extracted Frame");
}

void FSController::scan()
{
    if(webcam->info.portName.isEmpty()){
        mainwindow->showDialog("No webcam selected!");
        return;
    }
    QFuture<void> future = QtConcurrent::run(this, &FSController::scanThread);
}

void FSController::scanThread()
{
    if(webcam->info.portName.isEmpty()){
        mainwindow->showDialog("No webcam selected!");
        return;
    }
    this->detectLaserLine();
    this->laser->disable();

    scanning = true; //start scanning
    FSFloat stepDegrees = turntableStepSize;
    laser->turnOn();

    turntable->setDirection(FS_DIRECTION_CCW);
    turntable->enable();

    for(FSFloat i=0; i<360 && scanning==true; i+=stepDegrees){
        laser->turnOff();
        QThread::msleep(200);
        cv::Mat laserOff = webcam->getFrame();
        cv::resize( laserOff,laserOff,cv::Size(1280,960) );

        laser->turnOn();
        QThread::msleep(200);
        cv::Mat laserOn = webcam->getFrame();
        cv::resize( laserOn,laserOn,cv::Size(1280,960) );

        /*cv::namedWindow("extracted laserLine");
        cv::imshow("extracted laserLine",laserOff);
        cv::waitKey(0);
        cv::imshow("extracted laserLine",laserOn);
        cv::waitKey(0);
        cvDestroyWindow("extracted laserLine");*/

        vision->putPointsFromFrameToCloud(laserOff, laserOn, yDpi, 0);
        geometries->setPointCloudTo(model->pointCloud);
        mainwindow->redraw();
        turntable->turnNumberOfDegrees(stepDegrees);
        QThread::msleep(  300+stepDegrees*100);
    }
    if(scanning) mainwindow->doneScanning();
    scanning = false; //stop scanning
}

void FSController::scanThread2()
{
    if(webcam->info.portName.isEmpty()){
        mainwindow->showDialog("No webcam selected!");
        return;
    }
    scanning = true; //start scanning

    qDebug() << "done with turn to angle";
    //laser->turnNumberOfDegrees( laser->getRotation().y - LASER_SWIPE_MIN );
    turntable->setDirection(FS_DIRECTION_CW);
    for(FSFloat j=0; j<360 && scanning==true; j+=turntableStepSize){
        turntable->disable();
        laser->turnOn();
        laser->enable();
        laser->turnToAngle(laserSwipeMin);
        QThread::msleep(2500);
        laser->setDirection(FS_DIRECTION_CCW);
        for(FSFloat i=laserSwipeMin; i<laserSwipeMax && scanning==true; i+=laserStepSize){
            qDebug() << i;
            laser->turnOff();
            QThread::msleep(200);
            cv::Mat laserOff = webcam->getFrame();
            cv::resize( laserOff,laserOff,cv::Size(1280,960) );

            laser->turnOn();
            QThread::msleep(200);
            cv::Mat laserOn = webcam->getFrame();
            cv::resize( laserOn,laserOn,cv::Size(1280,960) );

            vision->putPointsFromFrameToCloud(laserOff, laserOn, 5, 0);
            geometries->setPointCloudTo(model->pointCloud);
            mainwindow->redraw();
            laser->turnNumberOfDegrees(laserStepSize);
            QThread::msleep(laserStepSize*100);
        }
        laser->disable();
        turntable->enable();
        turntable->turnNumberOfDegrees(turntableStepSize);
        std::string name;
        name.append(boost::lexical_cast<std::string>(j));
        name.append(".ply");
        //model->savePointCloudAsPLY(name);
        //model->pointCloud->clear();
        QThread::msleep(turntableStepSize*100);
    }
    if(scanning) mainwindow->doneScanning();
    scanning = false; //stop scanning
}

cv::Mat FSController::subLaser()
{
    laser->turnOff();
    QThread::msleep(200);
    cv::Mat laserOff = webcam->getFrame();
    cv::resize( laserOff,laserOff,cv::Size(1280,960) );

    laser->turnOn();
    QThread::msleep(200);
    cv::Mat laserOn = webcam->getFrame();
    cv::resize( laserOn,laserOn,cv::Size(1280,960) );

    return vision->subLaser(laserOff,laserOn,threshold);
}

cv::Mat FSController::diffImage()
{
    laser->turnOff();
    QThread::msleep(200);
    cv::Mat laserOff = webcam->getFrame();
    cv::resize( laserOff,laserOff,cv::Size(1280,960) );

    laser->turnOn();
    QThread::msleep(200);
    cv::Mat laserOn = webcam->getFrame();
    cv::resize( laserOn,laserOn,cv::Size(1280,960) );

    return vision->diffImage(laserOff,laserOn);
}

bool FSController::detectLaserLine()
{
    unsigned int threshold = 40;
    laser->turnOff();
    QThread::msleep(200);
    cv::Mat laserOffFrame = webcam->getFrame();
    laser->turnOn();
    QThread::msleep(200);
    cv::Mat laserOnFrame = webcam->getFrame();
    cv::resize( laserOnFrame,laserOnFrame,cv::Size(1280,960) );
    cv::resize( laserOffFrame,laserOffFrame,cv::Size(1280,960) );

    /*cv::imshow("extracted laserLine",laserOnFrame);
    cv::waitKey(0);
    cv::imshow("extracted laserLine",laserOffFrame);
    cv::waitKey(0);
    cvDestroyWindow("extracted laserLine");*/

    qDebug("images loaded, now detecting...");
    FSPoint p = vision->detectLaserLine( laserOffFrame, laserOnFrame, threshold );
    if(p.x == 0.0){return false;}
    laser->setLaserPointPosition(p);
    return true;
}

void FSController::computeSurfaceMesh()
{
    if(FSController::getInstance()->model->pointCloud->empty()){
        return;
    }
    //model->convertPointCloudToSurfaceMesh();
    //geometries->setSurfaceMeshTo(model->surfaceMesh,model->pointCloud);
    model->convertPointCloudToSurfaceMesh2();
    geometries->setSurfaceMeshTo(model->surfaceMeshPoisson,model->pointCloudPoisson);

    mainwindow->redraw();
}
