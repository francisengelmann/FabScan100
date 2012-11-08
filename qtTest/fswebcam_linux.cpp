/**
 * LINUX Implementation
 */

#include "fswebcam.h"

FSWebCam::FSWebCam()
{
    qDebug("linux constructor");
    enumerate();
}

FSWebCam::~FSWebCam()
{
    //platformSpecificDestructor();
}

FSWebCam::enumerate()
{
    QString program = "lsusb";
    QStringList arguments;
    // logitech c270
    // check for the listing in /var/lib/libutils/usb.ids
    arguments << "-d" << "046d:0825";

    QProcess proc;
    //proc.start("lsusb",QStringList() << "-d" << "046d:0825");
    proc.start("ls",QStringList() << "/dev/video*");
    if(!proc.waitForFinished()) return;

    QByteArray result = proc.readAll();
    qDebug(result);
}
