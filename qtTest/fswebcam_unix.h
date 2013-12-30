#ifndef FSWEBCAM_UNIX_H
#define FSWEBCAM_UNIX_H

#include <QStringList>
#include <QCamera>
#include <QCameraImageCapture>
#include <QApplication>
#include "fswebcam.h"
#include "fscontroller.h"
#include "ui_fscontrolpanel.h"
#include "staticHeaders.h"

class FSWebCamUnix : public FSWebCam
{
    //we need this macro, because this class has a SIGNAL: imageSaved
    Q_OBJECT

public:
    FSWebCamUnix();
    ~FSWebCamUnix();
    virtual cv::Mat getFrame();         //grab frame from camera and return as cv::Mat
    virtual void setCamera(const QByteArray &cameraDevice = 0);
    virtual QList<FSWebCamInfo> getCameras();

private slots:
    void imageSaved(int id, const QString &fileName);

private:
    cv::Mat frame;
    bool frameTaken;
    bool isCapturingImage;
    QCameraImageCapture* imageCapture;
    QImageEncoderSettings imageSettings;
    QCamera* camera;        //new qt5 camera representative
};

#endif // FSWEBCAM_UNIX_H
