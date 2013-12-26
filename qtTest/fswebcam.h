/**
 * This class controls the webcam and provides functionality
 * such as listing all available cams or grabbing images.
 */

#ifndef FSWEBCAM_H
#define FSWEBCAM_H

#include <QCamera>
#include <QCameraImageCapture>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <QFutureWatcher>



#include "fsgeometries.h"
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
#ifdef WINDOWS
	void StartX();			//Qt5 camera not fully operational in Windows. We use these two routines to sync cv capture
	void StartX2();
#endif
    cv::VideoCapture imageCaptureCv; 
    QImageEncoderSettings imageSettings;
    cv::Mat frame;
    bool frameTaken;
	bool endThread;

    FSWebCam();
    ~FSWebCam();
    //static QList<FSWebCamInfo> enumerate();
	void KillThread();

    cv::Mat getFrame();         //grab frame from camera and return as cv::Mat
    FSPoint getPosition(void);  //geometric position of hardware webcam inside scanner

    void setCamera(const QByteArray &cameraDevice);

signals:
	void cameraFrame(QImage frame);

private slots:

    void processCapturedImage(int requestId, const QImage& img);
    void imageSaved(int id, const QString &fileName);

private:
    bool isCapturingImage;
	QImage Mat2QImage(const cv::Mat3b &src);
	QFutureWatcher<void> watcher;
};

void debug_dialog(char *message);

#endif // FSWEBCAM_H
