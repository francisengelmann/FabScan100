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

cv::Mat FSWebCam::getFrame()
{
    cv::VideoCapture capture(-1);
    cv::Mat frame;
    if(!capture.isOpened()) return frame;
    capture.read(frame);
    capture.release();
    return frame;
}

QList<FSWebCamInfo> FSWebCam::enumerate()
{
    QList<FSWebCamInfo> list;
    return list;
    qDebug("Not yet implemented.");
}
