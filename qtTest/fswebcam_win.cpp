#include "fswebcam_win.h"

FSWebCamWin::FSWebCamWin()
{
    imageCaptureCv = 0;
    connectedImageReceivers = 0;
}

FSWebCamWin::~FSWebCamWin()
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

void FSWebCamWin::connectNotify(const QMetaMethod &signal)
{
    // count the connected frame receivers
    if (signal == QMetaMethod::fromSignal(&FSWebCamWin::cameraFrame)) {
        connectedImageReceivers++;
        qDebug() << "Image receiver connected. Now: " << connectedImageReceivers;
    }
}

void FSWebCamWin::disconnectNotify(const QMetaMethod &signal)
{
    // count the connected frame receivers
    if (signal == QMetaMethod::fromSignal(&FSWebCamWin::cameraFrame)) {
        connectedImageReceivers--;
        qDebug() << "Image receiver disconnected. Now: " << connectedImageReceivers;
    }
}

// These routines take the place of getFrame().
void FSWebCamWin::StartX()	// This sub starts StartX2() as a seperate thread
{
    endThread = false;
    QFuture<void> future = QtConcurrent::run(this, &FSWebCamWin::StartX2);
    watcher.setFuture(future);
}

void FSWebCamWin::StartX2() // This thread runs concurrently
{
    cv::Mat capture;
    while(!endThread) // endThread is set when the main program wishes to close this thread
    {
        QThread::msleep(10);
        // don't do anything if noone is interested
        if(!isCapturingImage || connectedImageReceivers <= 0) continue;

        imageCaptureCv.read(capture); // = CV VideoCapture::read() (Grab, decode and return frame)
        // Stores grab locally - we can't modify!
        frameTaken = false;
        QThread::msleep(1);
        frame=capture.clone();		// Copy grab into global 'frame' of type MAT
        //			QThread::msleep(1);		// If we're going to transmit to controlpanel->ui->viewfinder this is where we'd do it
        // It will need to be 'massaged' because MAT is BGR and QImage is RGB - see Mat2QImage() below
        // We can send to FSControlPanel->ui->cameraLabel
        if(connectedImageReceivers > 0) // emit the frame only if anyone is listening (converting causes high cpu load)
            emit cameraFrame(Mat2QImage(capture)); // Convert to QImage and send to fscontrolpanel
        frameTaken = true;
    }
    qDebug() << "Thread killed";
    endThread = false; // Signal back to the main program that we've finished
}

// Can we use the below code in the above thread to create a QImage usable in ui->viewfinder? Do we need to?
QImage FSWebCamWin::Mat2QImage(const cv::Mat3b &src)
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

void FSWebCamWin::KillThread()  // Looks like we have to wait for thread to actually end before doing any more destructing!
{
    endThread = true;
    watcher.cancel();

    watcher.waitForFinished();
}

cv::Mat FSWebCamWin::getFrame()
{
    // The OpenCV routines run as concurrent threads, so all we need to do is set
    frameTaken = false;
    isCapturingImage = true;

    qDebug() << "preparing to take frame";
    //wait until camera has taken picture, then return */

    while(!frameTaken){
        QThread::msleep(1); // Added for the cv routines
        qApp->processEvents();
    }

    qDebug() << "received frame";
    //return frame.clone();

    cv::Mat result = frame.clone();
    cv::imshow(WINDOW_EXTRACTED_FRAME,result);

    return result;
}

void FSWebCamWin::setCamera(const QByteArray &cameraDevice)
{
    Q_UNUSED(cameraDevice);

    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return;

    imageCaptureCv = cap;

    imageCaptureCv.set( CV_CAP_PROP_FPS, 15);
    imageCaptureCv.set( CV_CAP_PROP_FRAME_WIDTH, this->info.sizeX);
    imageCaptureCv.set(CV_CAP_PROP_FRAME_HEIGHT, this->info.sizeY);

    this->info.portName = "DefaultCamera";
    this->StartX();
    qDebug() << "Default camera selected";
}

QList<FSWebCamInfo> FSWebCamWin::getCameras()
{
    QList<FSWebCamInfo> result;
    if(QCamera::availableDevices().size()==0)
        return result;

    // As long as no camera enumeration in OpenCV is possible:
    FSWebCamInfo info;
    info.portName = "DefaultCamera";
    info.friendlyName = "Default Camera";
    info.deviceName = QVariant("DefaultCamera");
    result.append(info);
    return result;
}


