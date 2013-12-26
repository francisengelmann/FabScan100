#include "fswebcam.h"
#include "ui_fscontrolpanel.h"
#include "fscontroller.h"
#include <QProcess>
#include <QCameraViewfinder>
#include <QObject>

#ifdef WINDOWS
#include <QFuture>			//These are required for the OpenCV capture thread, which replaces the non-functional...
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun> // ...Windows version of QCameraImageCapture
#endif
FSWebCam::FSWebCam()
{
    info.portName = "";
    info.friendlyName = "";
    info.sizeX = FSController::config->CAM_IMAGE_WIDTH;
    info.sizeY = FSController::config->CAM_IMAGE_HEIGHT;
    isCapturingImage=false;
    camera=0;
    imageCapture=0;

#ifdef WINDOWS
	//Next is added for the cv capture replacement...	 
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
	{
		qDebug() << "Cam device not open";
        return;
	}

	imageCaptureCv = cap;
	 	 
	imageCaptureCv.set( CV_CAP_PROP_FPS, 15);
    imageCaptureCv.set( CV_CAP_PROP_FRAME_WIDTH, 1280);
    imageCaptureCv.set(CV_CAP_PROP_FRAME_HEIGHT, 960);
#endif
}

FSWebCam::~FSWebCam()
{
    KillThread(); //This successfully disconnects the Logitech driver
	if(imageCaptureCv.isOpened()){
		qDebug() << "cap is opened";
		imageCaptureCv.release();
		qDebug() << "Capture released";
	}
	else {
		qDebug() << "cap is closed";
	}
}

#ifdef WINDOWS
// These routines take the place of getFrame().
void FSWebCam::StartX()	// This sub starts StartX2() as a seperate thread
{
//	connect (&watcher, SIGNAL(finished()), this, SLOT(handleFinished()));
	endThread = false;
	QFuture<void> future = QtConcurrent::run(this, &FSWebCam::StartX2);
	watcher.setFuture(future);
}

void FSWebCam::StartX2() // This thread runs concurrently
{ 
	Mat NN;
	while(!endThread) // endThread is set when the main program wishes to close this thread
	{
		 
			imageCaptureCv.read(NN); // = CV VideoCapture::read() (Grab, decode and return frame)
									 // Stores grab locally - we can't modify!
			  frameTaken = false;
			  QThread::msleep(1);
			frame=NN.clone();		// Copy grab into global 'frame' of type MAT
//			QThread::msleep(1);		// If we're going to transmit to controlpanel->ui->viewfinder this is where we'd do it
									// It will need to be 'massaged' because MAT is BGR and QImage is RGB - see Mat2QImage() below
									// We can send to FSControlPanel->ui->cameraLabel
			emit cameraFrame(Mat2QImage(NN)); // Convert to QImage and send to fscontrolpanel
			   frameTaken = true;
		QThread::msleep(10);

	}
	qDebug() << "Thread killed";
	endThread = false; // Signal back to the main program that we've finished
}
#endif

// Can we use the below code in the above thread to create a QImage usable in ui->viewfinder? Do we need to?
QImage FSWebCam::Mat2QImage(const cv::Mat3b &src)
{
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3b *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
                }
        }
        return dest;
}

void FSWebCam::KillThread()  // Looks like we have to wait for thread to actually end before doing any more destructing!
{
	endThread = true;
	watcher.cancel();

	watcher.waitForFinished();
}

cv::Mat FSWebCam::getFrame()
{
// The OpenCV routines run as concurrent threads, so all we need to do is set
    frameTaken = false;
    isCapturingImage = true;

    #ifdef LINUX
    imageCapture->capture();
    #elif MACOSX
    imageCapture->capture("./");
    #endif
    qDebug() << "preparing to take frame";
    //wait until camera has taken picture, then return
    while(!frameTaken){
#ifdef WINDOWS
		QThread::msleep(1); // Added for the cv routines
#endif
        qApp->processEvents();
    }
    qDebug() << "received frame";

#if WINDOWS
	Mat ONOFF = frame.clone();
	cv::imshow("Extracted Frame",ONOFF);/* waitKey(1) */;
	return ONOFF;
#else
	return frame.clone();
#endif
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
	qDebug() << "Camera descriptor = " << cameraDevice;
    delete camera;
    delete imageCapture;
    if (cameraDevice.isEmpty()){
        camera = new QCamera;
    }else{
        camera = new QCamera(cameraDevice);
    }
#ifndef WINDOWS
    imageCapture = new QCameraImageCapture(camera);
    QImageEncoderSettings imageSettings = imageCapture->encodingSettings();
        qDebug() << imageCapture->supportedResolutions();
        qDebug() << imageSettings.resolution();
    //QImageEncoderSettings imageSettings;
#endif

    #ifndef LINUX
    imageSettings.setCodec("image/jpeg");
    imageSettings.setResolution(1280, 960);
    #endif

#ifndef WINDOWS
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
#endif
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
