#include "fscontroller.h"
#include "fsdialog.h"

#include <opencv2/imgproc/imgproc.hpp>

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
    cv::resize(frame,frame,cv::Size(400,300));
    cv::imshow("Extracted Frame",frame);
    cv::waitKey(0);
    cvDestroyWindow("Extracted Frame");
}

void FSController::hideFrame()
{
    cvDestroyWindow("Extracted Frame");
}

void FSController::detectLaserLine()
{
    cv::Mat laserOn = cv::imread("../../../laser_on.jpg");
    cv::Mat laserOff = cv::imread("../../../laser_off.jpg");
    cv::resize(laserOn,laserOn,cv::Size(1280,960));
    cv::resize(laserOff,laserOff,cv::Size(1280,960));
    //cv::Mat result = vision->subLaser(laserOff,laserOn,100);
    cv::Mat result = vision->drawHelperLinesToFrame(laserOn);
    result = vision->drawLaserLineToFrame(laserOn);

    cv::resize(result,result,cv::Size(800,600));
    cv::imshow("Laser Frame",result);
    cv::waitKey(0);
    cvDestroyWindow("Laser Frame");

    qDebug("detectLaserLiner");
}
