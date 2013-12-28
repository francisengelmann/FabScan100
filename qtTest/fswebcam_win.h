#ifndef FSWEBCAM_WIN_H
#define FSWEBCAM_WIN_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QStringList>
#include <QImage>
#include <QFuture>			//These are required for the OpenCV capture thread, which replaces the non-functional...
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun> // ...Windows version of QCameraImageCapture
#include <QProcess>
#include <QObject>
#include <QApplication>

#include "fswebcam.h"
#include "fscontroller.h"
#include "staticHeaders.h"
#include <QCamera>

class FSWebCamWin : public FSWebCam
{
public:
    FSWebCamWin();
    ~FSWebCamWin();
    virtual cv::Mat getFrame();         //grab frame from camera and return as cv::Mat
    virtual void setCamera(const QByteArray &cameraDevice = 0);
    virtual QList<FSWebCamInfo> getCameras();

protected:
    void connectNotify(const QMetaMethod &signal);
    void disconnectNotify(const QMetaMethod &signal);

private:
    void StartX();			//Qt5 camera not fully operational in Windows. We use these two routines to sync cv capture
    void StartX2();

    QImage Mat2QImage(const cv::Mat3b &src);
    void KillThread();

    cv::VideoCapture imageCaptureCv;
    cv::Mat frame;
    bool frameTaken;
    bool endThread;
    bool isCapturingImage;
    int connectedImageReceivers;

    QFutureWatcher<void> watcher;
};

#endif // FSWEBCAM_WIN_H
