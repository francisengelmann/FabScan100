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
    QList<FSWebCamInfo> list;
    return list;
    qDebug("Not yet implemented.");
}
