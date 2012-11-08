/**
 * OSX Implementation
 */

#include "fswebcam.h"

FSWebCam::FSWebCam()
{
    qDebug("mac constructor");
}

FSWebCam::~FSWebCam()
{
    //platformSpecificDestructor();
}

QList<FSWebCamInfo> FSWebCam::enumerate()
{
    qDebug("Not yet implemented.");
}
