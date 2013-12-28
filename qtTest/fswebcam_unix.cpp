#include "fswebcam_unix.h"

FSWebCamUnix::FSWebCamUnix()
{
    camera=0;
    imageCapture=0;
    isCapturingImage=false;
}

FSWebCamUnix::~FSWebCamUnix()
{
    delete imageCapture;
    delete camera;
}

cv::Mat FSWebCamUnix::getFrame()
{
    frameTaken = false;
    isCapturingImage = true;

    #ifdef MACOSX
    imageCapture->capture("./");
    #elif LINUX
    imageCapture->capture();
    #endif
    //qDebug() << "preparing to take frame";
    //wait until camera has taken picture, then return
    while(!frameTaken){
        qApp->processEvents();
    }
    //qDebug() << "received frame";
    return frame.clone();
}

void FSWebCamUnix::setCamera(const QByteArray &cameraDevice)
{
    delete camera;
    delete imageCapture;
    if (cameraDevice.isEmpty()){
        camera = new QCamera;
    }else{
        camera = new QCamera(cameraDevice);
    }

    camera->setViewfinder(FSController::getInstance()->controlPanel->ui->viewfinder );
    camera->setCaptureMode(QCamera::CaptureStillImage);

    imageCapture = new QCameraImageCapture(camera);
    QImageEncoderSettings imageSettings = imageCapture->encodingSettings();
    #ifdef MACOSX
        imageSettings.setCodec("image/jpeg");
        imageSettings.setResolution(1280, 960);
        imageCapture->setEncodingSettings(imageSettings);
    #endif

    connect(imageCapture, SIGNAL(imageSaved(int,QString)), this, SLOT(imageSaved(int,QString)));

    camera->start();

    // Debug info
    qDebug() << "Camera:" << cameraDevice;
    qDebug() << "Selected Resoloution: " << imageSettings.resolution().width() << " x " << imageSettings.resolution().height();

    qDebug() << "Supported Resoloutions (" << imageCapture->supportedResolutions().count() << "):";
    foreach(QSize r, imageCapture->supportedResolutions()){ qDebug() << "\tSupported Resoloution: "<< r.width() << " x " << r.height(); }

    qDebug() << "Supported Codecs (" << imageCapture->supportedImageCodecs().count() << "):";
    foreach(QString c, imageCapture->supportedImageCodecs()){ qDebug() << "\tSupported Codec: " << c; }

    qDebug() << "Supported BufferFormats (" << imageCapture->supportedBufferFormats().count() << "):";
    foreach(QVideoFrame::PixelFormat b, imageCapture->supportedBufferFormats()){ qDebug() << "\tSupported BufferFormat: " << b; }

    FSController::getInstance()->controlPanel->ui->cameraLabel->setStyleSheet("border-style: solid; border-color: black; border-width: 3px 1px 3px 1px;");
    FSController::getInstance()->controlPanel->ui->cameraLabel->setText("");
}

void FSWebCamUnix::imageSaved(int id, const QString &fileName)
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
    #ifdef MACOSX
    frame = mat;
    #elif LINUX
    frame = mat.clone();
    #endif
    cv::cvtColor(mat,frame, CV_RGB2BGR);
    frameTaken = true;
    isCapturingImage = false;
}

QList<FSWebCamInfo> FSWebCamUnix::getCameras()
{
    QList<FSWebCamInfo> result;
    if(QCamera::availableDevices().size()==0)
        return result;

    foreach(const QByteArray &deviceName, QCamera::availableDevices()) {
        FSWebCamInfo info;
        info.friendlyName = QCamera::deviceDescription(deviceName);
        info.deviceName = QVariant(deviceName);
        result.append(info);
    }

    return result;
}
