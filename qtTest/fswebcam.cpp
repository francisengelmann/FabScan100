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
    isCapturingImage=false;
    camera=0;
    imageCapture=0;
}

FSWebCam::~FSWebCam()
{
    delete imageCapture;
    delete camera;
}

cv::Mat FSWebCam::getFrame()
{
    frameTaken = false;
    isCapturingImage = true;
    imageCapture->capture();
    //qDebug() << "preparing to take frame";
    //wait until camera has taken picture, then return
    while(!frameTaken){
        qApp->processEvents();
    }
    //qDebug() << "received frame";
    return frame.clone();
}

FSPoint FSWebCam::getPosition()
{
    return FSMakePoint(CAM_POS_X, CAM_POS_Y, CAM_POS_Z);
}

void FSWebCam::setCamera(const QByteArray &cameraDevice)
{
    qDebug() << "setCamera...";
    delete camera;
    delete imageCapture;
    qDebug() << "deleted old stuff Camera...";
    if (cameraDevice.isEmpty()){
        qDebug() << cameraDevice << "cameraDevice empty";
        camera = new QCamera;
    }else{
        qDebug() << cameraDevice << "cameraDevice not empty";
        camera = new QCamera(cameraDevice);
    }
    qDebug() << camera;
    imageCapture = new QCameraImageCapture(camera);
    qDebug() << imageCapture;
    camera->setViewfinder(FSController::getInstance()->controlPanel->ui->viewfinder );
    FSController::getInstance()->controlPanel->ui->cameraLabel->setStyleSheet("border-style: solid; border-color: black; border-width: 3px 1px 3px 1px;");
    FSController::getInstance()->controlPanel->ui->cameraLabel->setText("");
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
