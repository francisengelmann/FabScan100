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
    info.sizeX = FSController::config->CAM_IMAGE_WIDTH;
    info.sizeY = FSController::config->CAM_IMAGE_HEIGHT;
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

    #ifdef LINUX
    imageCapture->capture();
    #else
    imageCapture->capture("./");
    #endif
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
    //cout << __PRETTY_FUNCTION__
    //        << FSController::config->CAM_POS_X << "::"
    //        << FSController::config->CAM_POS_Y << "::"
     //       << FSController::config->CAM_POS_Z << endl;
    return FSMakePoint(FSController::config->CAM_POS_X,
                       FSController::config->CAM_POS_Y,
                       FSController::config->CAM_POS_Z);
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
    QImageEncoderSettings imageSettings = imageCapture->encodingSettings();
        qDebug() << imageCapture->supportedResolutions();
        qDebug() << imageSettings.resolution();
    //QImageEncoderSettings imageSettings;

    #ifdef LINUX
    #else
    imageSettings.setCodec("image/jpeg");
    imageSettings.setResolution(1280, 960);
    #endif

    imageCapture->setEncodingSettings(imageSettings);

    camera->setViewfinder(FSController::getInstance()->controlPanel->ui->viewfinder );
    FSController::getInstance()->controlPanel->ui->cameraLabel->setStyleSheet("border-style: solid; border-color: black; border-width: 3px 1px 3px 1px;");
    FSController::getInstance()->controlPanel->ui->cameraLabel->setText("");
    camera->setCaptureMode(QCamera::CaptureStillImage);

    //connect(imageCapture, SIGNAL(readyForCaptureChanged(bool)), this, SLOT(readyForCapture(bool)));
    connect(imageCapture, SIGNAL(imageCaptured(int,QImage)), this, SLOT(processCapturedImage(int,QImage)));
    connect(imageCapture, SIGNAL(imageSaved(int,QString)), this, SLOT(imageSaved(int,QString)));

    camera->start();
    QStringList res = imageCapture->supportedImageCodecs();
    //QList<QSize> res = imageCapture->supportedResolutions();
    //QSize r;
    QString r;
    foreach(r,res){
        qDebug() << "Supported Resolutions:"<<r;
    }
    qDebug() << "set camera:" << cameraDevice;
}

void FSWebCam::processCapturedImage(int requestId, const QImage& img)
{
    Q_UNUSED(requestId);
    Q_UNUSED(img);
    //qDebug("processing captured frame");
    //remove alpha channel
    /*QImage img2 = img.convertToFormat(QImage::Format_RGB888);
    qDebug() << img2.height() << img2.width();
    cv::Mat mat(img2.height(),
                img2.width(),
                CV_8UC3,
                (uchar*)img2.bits(), img2.bytesPerLine());
    frame = mat;
    cv::cvtColor(mat,frame, CV_RGB2BGR);
    frameTaken = true;*/
}

void FSWebCam::imageSaved(int id, const QString &fileName)
{
    Q_UNUSED(id);
    Q_UNUSED(fileName);
    QImage img = QImage(fileName);
    QFile::remove(fileName);
    QImage img2 = img.convertToFormat(QImage::Format_RGB888);
    //qDebug() << img2.height() << img2.width();
    cv::Mat mat(img2.height(),
                img2.width(),
                CV_8UC3,
                (uchar*)img2.bits(), img2.bytesPerLine());
    #ifdef LINUX
    frame = mat.clone();
    #else
    frame = mat;
    #endif
    cv::cvtColor(mat,frame, CV_RGB2BGR);
    frameTaken = true;
    isCapturingImage = false;
    //if (applicationExiting)
        //close();
}
