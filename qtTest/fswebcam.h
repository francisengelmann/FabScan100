/**
 * This class controls the webcam and provides functionality
 * such as listing all available cams or grabbing images.
 */

#ifndef FSWEBCAM_H
#define FSWEBCAM_H

#include <QCamera>
#include <QCameraImageCapture>
#include "staticHeaders.h"

struct FSWebCamInfo
{
    QString portName;       //path to the webcam e.g. /dev/video0
    QString friendlyName;   //name of webcam that is displayed
    int sizeX;
    int sizeY;
};

class FSWebCam : public QObject
{
    Q_OBJECT

public:
    FSWebCamInfo info;      //the string that identifies the camera, as selected in the menu
    QCamera* camera;        //new qt5 camera representative
    QCameraImageCapture *imageCapture;
    QImageEncoderSettings imageSettings;
    cv::Mat frame;
    bool frameTaken;

    FSWebCam();
    ~FSWebCam();
    //static QList<FSWebCamInfo> enumerate();

    cv::Mat getFrame();         //grab frame from camera and return as cv::Mat
    FSPoint getPosition(void);  //geometric position of hardware webcam inside scanner

    void setCamera(const QByteArray &cameraDevice);

private slots:

    void processCapturedImage(int requestId, const QImage& img);
    void imageSaved(int id, const QString &fileName);

private:
    bool isCapturingImage;

};

#endif // FSWEBCAM_H
