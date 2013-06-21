#include "fscontroller.h"
#include "fsdialog.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace std;

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
    //all in degrees; (only when stepper is attached to laser)
    laserSwipeMin = 30; //18
    laserSwipeMax = 45; //50
    meshComputed = false;
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
    cv::resize(result,result,cv::Size(800,600)); //this is the resolution of the preview
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
    //check if camera is connected
    if(webcam->info.portName.isEmpty()){
        mainwindow->showDialog("No webcam selected!");
        return;
    }
    //detect laser line
    this->detectLaserLine();
    //turn off stepper (if available)
    this->laser->disable();

    scanning = true; //start scanning, if false, scan stops
    FSFloat stepDegrees = turntableStepSize;

    laser->turnOn();
    turntable->setDirection(FS_DIRECTION_CCW);
    turntable->enable();

    //iterate over a complete turn of the turntable
    for(FSFloat i=0; i<360 && scanning==true; i+=stepDegrees){
        //take picture without laser
        laser->turnOff();
        QThread::msleep(200);
        cv::Mat laserOff = webcam->getFrame();
        cv::resize( laserOff,laserOff,cv::Size(1280,960) );

        //take picture with laser
        laser->turnOn();
        QThread::msleep(200);
        cv::Mat laserOn = webcam->getFrame();
        cv::resize( laserOn,laserOn,cv::Size(1280,960) );

        //here the magic happens
        vision->putPointsFromFrameToCloud(laserOff, laserOn, yDpi, 0);
        //update gui
        geometries->setPointCloudTo(model->pointCloud);
        mainwindow->redraw();
        //turn turntable a step
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

    qDebug("images loaded, now detecting...");
    FSPoint p = vision->detectLaserLine( laserOffFrame, laserOnFrame, threshold );
    if(p.x == 0.0){return false;}
    laser->setLaserPointPosition(p);
    return true;
}

void FSController::computeSurfaceMesh()
{
    //model->convertPointCloudToSurfaceMesh();
    //geometries->setSurfaceMeshTo(model->surfaceMesh,model->pointCloud);
    model->convertPointCloudToSurfaceMesh2();
    model->convertPolygons2Triangles();
    //cout << "FSController:computesurfaceMesh: convert done, now setting" << endl;

    //geometries->setSurfaceMeshTo(model->surfaceMesh,model->pointCloud);

    //mainwindow->redraw();
}
