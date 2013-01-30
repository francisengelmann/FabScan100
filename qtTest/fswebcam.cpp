#include "fswebcam.h"
#include "ui_fscontrolpanel.h"
#include "fscontroller.h"
#include <QProcess>
#include <QCameraViewfinder>
#include <QObject>

FSWebCam::FSWebCam()
{

    info.portName = "";
    info.friendlyName = "";
    info.sizeX = CAM_IMAGE_WIDTH;
    info.sizeY = CAM_IMAGE_HEIGHT;
    //enumerate();
    isCapturingImage=false;
}

FSWebCam::~FSWebCam()
{
    //platformSpecificDestructor();
    camera->stop();
    delete imageCapture;
    delete camera;
}

cv::Mat FSWebCam::getFrame()
{
    frameTaken = false;
    isCapturingImage = true;
    //frame.data=0;
    imageCapture->capture();
    //qDebug() << "preparing to take frame";
    //wait until camera has taken picture, then return
    while(!frameTaken){
        qApp->processEvents();
    }
    //qDebug() << "received frame";
    //frame = cv::imread("/Users/francis/bild.png");
    return frame.clone();
}

FSPoint FSWebCam::getPosition()
{
    return FSMakePoint(CAM_POS_X, CAM_POS_Y, CAM_POS_Z);
}

void FSWebCam::setCamera(const QByteArray &cameraDevice)
{
    delete camera;
    delete imageCapture;

    if (cameraDevice.isEmpty()){
        camera = new QCamera;
    }else{
        camera = new QCamera(cameraDevice);
    }

    imageCapture = new QCameraImageCapture(camera);
    camera->setViewfinder(FSController::getInstance()->controlPanel->ui->viewfinder );
    camera->setCaptureMode(QCamera::CaptureStillImage);

    //connect(imageCapture, SIGNAL(readyForCaptureChanged(bool)), this, SLOT(readyForCapture(bool)));
    connect(imageCapture, SIGNAL(imageCaptured(int,QImage)), this, SLOT(processCapturedImage(int,QImage)));
    connect(imageCapture, SIGNAL(imageSaved(int,QString)), this, SLOT(imageSaved(int,QString)));

    qDebug() << "set camera:" << cameraDevice;
    camera->start();
}

void FSWebCam::processCapturedImage(int requestId, const QImage& img)
{
    Q_UNUSED(requestId);
    //qDebug("processing captured frame");
    //remove alpha channel
    QImage img2 = img.convertToFormat(QImage::Format_RGB888);

    cv::Mat mat(img2.height(),
                img2.width(),
                CV_8UC3,
                (uchar*)img2.bits(), img2.bytesPerLine());
    frame = mat;
    cv::cvtColor(mat,frame, CV_RGB2BGR);
    frameTaken = true;
}

void FSWebCam::imageSaved(int id, const QString &fileName)
{
    Q_UNUSED(id);
    Q_UNUSED(fileName);

    isCapturingImage = false;
    //if (applicationExiting)
        //close();
}
