#include "fscontroller.h"
#include "fsdialog.h"
#include "FSDefines.h"

#include <opencv2/imgproc/imgproc.hpp>

FSController* FSController::singleton=0;

FSController::FSController()
{
    model = new FSModel();
    geometries = new GeometryEngine();
    serial = new FSSerial();
    webcam = new FSWebCam();
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

void FSController::turnLaserOn()
{
    qDebug("turnLaserOn");
    serial->writeChar(MC_TURN_LASER_ON);
}

void FSController::turnLaserOff(){
    qDebug("turnLaserOff");
    serial->writeChar(MC_TURN_LASER_OFF);
}

void FSController::detectLaserLine()
{
    qDebug("detectLaserLiner");
}

void FSController::turnStepperOn()
{
    qDebug("turnStepperOn");
    serial->writeChar(MC_TURN_STEPPER_ON);
}

void FSController::turnStepperOff()
{
    qDebug("turnStepperOff");
    serial->writeChar(MC_TURN_STEPPER_OFF);
}

void FSController::performStep()
{
    qDebug("performStep");
    serial->writeChar(MC_PERFORM_STEP);
}

void FSController::setDirectionCW()
{
    qDebug("setDirectionCW");
    serial->writeChar(MC_SET_DIRECTION_CW);
}

void FSController::setDirectionCCW()
{
    qDebug("setDirectionCCW");
    serial->writeChar(MC_SET_DIRECTION_CCW);
}
