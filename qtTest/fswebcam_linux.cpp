/**
 * LINUX Implementation
 */

#include "fswebcam.h"
#include <QProcess>

FSWebCam::FSWebCam()
{
    info.portName = "";
    info.friendlyName = "";
    info.sizeX = CAM_IMAGE_WIDTH;
    info.sizeY = CAM_IMAGE_HEIGHT;
    //enumerate();
}

FSWebCam::~FSWebCam()
{
    //platformSpecificDestructor();
}

cv::Mat FSWebCam::getFrame()
{
    //all this is still quite slow, but works
    cv::Mat frame;
    QProcess proc;
    //uvccapture -d/dev/video0 -x1280 -y960 -otest.jpg
    QString portName = "-d";
    portName.append(info.portName);
    QString sizeX = "-x";
    sizeX.append(QString::number(info.sizeX));
    QString sizeY = "-y";
    sizeY.append(QString::number(info.sizeY));
    proc.start("uvccapture",QStringList()<<portName<<sizeX<<sizeY<<"-oshot.jpg");
    proc.waitForFinished();
    frame = cv::imread("shot.jpg");
    return frame.clone();
}

QList<FSWebCamInfo> FSWebCam::enumerate()
{
    QString program = "lsusb";
    //QStringList arguments;
    // logitech c270
    // check for the listing in /var/lib/libutils/usb.ids
    //arguments << "-d" << "046d:0825";

    QList<FSWebCamInfo> list;
    QProcess proc;
    //proc.start("lsusb",QStringList() << "-d" << "046d:0825");
    proc.start("ls",QStringList() << "/dev");
    if(!proc.waitForFinished()) return list;
    QByteArray result = proc.readAll();
    QList<QByteArray> results = result.split('\n');
    foreach(QByteArray device, results){
        if(device.startsWith("video")){
            FSWebCamInfo cam;
            cam.portName = "/dev/";
            cam.portName.append(device);
            list.append(cam);
        }
    }
    return list;
}

FSPoint FSWebCam::getPosition()
{
    return FSMakePoint(CAM_POS_X, CAM_POS_Y, CAM_POS_Z);
}
