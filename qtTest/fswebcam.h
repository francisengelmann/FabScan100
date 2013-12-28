/**
 * This class controls the webcam and provides functionality
 * such as listing all available cams or grabbing images.
 */

#ifndef FSWEBCAM_H
#define FSWEBCAM_H

#include <QImage>
#include <QList>
#include "staticHeaders.h"

struct FSWebCamInfo
{
    QString portName;       //path to the webcam e.g. /dev/video0
    QString friendlyName;   //name of webcam that is displayed
    QVariant deviceName;
    int sizeX;
    int sizeY;
};

class FSWebCam : public QObject
{
    Q_OBJECT

public:
    FSWebCamInfo info;      //the string that identifies the camera, as selected in the menu

    FSWebCam();
    ~FSWebCam();

    FSPoint getPosition(void);  //geometric position of hardware webcam inside scanner

    virtual cv::Mat getFrame() = 0;         //grab frame from camera and return as cv::Mat
    virtual void setCamera(const QByteArray &cameraDevice = 0) = 0;
    virtual QList<FSWebCamInfo> getCameras() = 0;
signals:
    void cameraFrame(QImage frame);
};

#endif // FSWEBCAM_H
