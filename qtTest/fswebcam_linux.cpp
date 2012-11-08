/**
 * LINUX Implementation
 */

#include "fswebcam.h"
#include <QProcess>

FSWebCam::FSWebCam()
{
    qDebug("linux constructor");
    enumerate();
}

FSWebCam::~FSWebCam()
{
    //platformSpecificDestructor();
}

QList<FSWebCamInfo> FSWebCam::enumerate()
{
    QList<FSWebCamInfo> list;
    QString program = "lsusb";
    QStringList arguments;
    // logitech c270
    // check for the listing in /var/lib/libutils/usb.ids
    arguments << "-d" << "046d:0825";

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
            //qDebug() << "Result: " << path;
        }
    }
    return list;
}
