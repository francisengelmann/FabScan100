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
    cv::resize(frame,frame,cv::Size(1280,960));
    cv::Mat result = vision->drawHelperLinesToFrame(frame);
    //cv::Mat blub = vision->drawLaserLineToFrame(result);
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
    laser->turnOff();
    cv::Mat laserOff = webcam->getFrame();
    laser->turnOn();
    cv::Mat laserOn = webcam->getFrame();
    //cv::Mat laserOn = cv::imread("../../../laser_on.jpg");
    //cv::Mat laserOff = cv::imread("../../../laser_off.jpg");
    cv::resize(laserOn,laserOn,cv::Size(1280,960));
    cv::resize(laserOff,laserOff,cv::Size(1280,960));
    //cv::Mat subLaser = vision->subLaser(laserOff,laserOn,100);
    //cv::Mat result = vision->drawHelperLinesToFrame(laserOn);
    //result = vision->drawLaserLineToFrame(laserOn);
    cv::imshow("Laser Frame",laserOn);
    cv::waitKey(0);
    cv::imshow("Laser Frame",laserOff);
    cv::waitKey(0);
    cvDestroyWindow("Laser Frame");

    vision->putPointsFromFrameToCloud(laserOff,laserOn,5,0,100);
    FSController::getInstance()->geometries->setPointCloudTo(
                FSController::getInstance()->model->pointCloud
                );

    /*cv::resize(result,result,cv::Size(800,600));
    cv::imshow("Laser Frame",result);
    cv::waitKey(0);*/
}

void FSController::detectLaserLine()
{
    laser->turnOff();
    cv::Mat laserOffFrame = webcam->getFrame();
    laser->turnOn();
    cv::Mat laserOnFrame = webcam->getFrame();
    //laser->turnOff();
    //cv::Mat laserOnFrame = cv::imread("../../../laser_on.jpg");
    //cv::Mat laserOffFrame = cv::imread("../../../laser_off.jpg");
    cv::resize( laserOnFrame,laserOnFrame,cv::Size(1280,960) );
    cv::resize( laserOffFrame,laserOffFrame,cv::Size(1280,960) );

    /*cv::imshow("Detected Lines with HoughP",laserOffFrame);
    cv::waitKey(0);
    cv::imshow("Detected Lines with HoughP",laserOnFrame);
    cv::waitKey(0);
    cvDestroyWindow("Detected Lines with HoughP");*/

    qDebug("images loaded, now detecting...");
    FSPoint p = vision->detectLaserLine( laserOffFrame, laserOnFrame, 40 );
    laser->setLaserPointPosition(p);
    //laser->setRotation(); ??
}

void FSController::computeSurfaceMesh()
{
    if(FSController::getInstance()->model->pointCloud->empty()){
        return;
    }
    //qDebug("converting...");
    model->convertPointCloudToSurfaceMesh();
    geometries->setSurfaceMeshTo(model->triangles,model->pointCloud);
}
