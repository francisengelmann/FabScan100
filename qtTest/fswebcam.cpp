#include "fswebcam.h"
#include "fscontroller.h"

FSWebCam::FSWebCam()
{
    info.portName = "";
    info.friendlyName = "";
    info.sizeX = FSController::config->CAM_IMAGE_WIDTH;
    info.sizeY = FSController::config->CAM_IMAGE_HEIGHT;
}

FSWebCam::~FSWebCam(){}


FSPoint FSWebCam::getPosition()
{
    return FSMakePoint(FSController::config->CAM_POS_X,
                       FSController::config->CAM_POS_Y,
                       FSController::config->CAM_POS_Z);
}


