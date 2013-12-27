#include "fswebcam_win.h"

FSWebCamWin::FSWebCamWin()
{
    imageCaptureCv = 0;
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

// These routines take the place of getFrame().
void FSWebCamWin::StartX()	// This sub starts StartX2() as a seperate thread
{
    endThread = false;
    QFuture<void> future = QtConcurrent::run(this, &FSWebCamWin::StartX2);
    watcher.setFuture(future);
}

void FSWebCamWin::StartX2() // This thread runs concurrently
{
    cv::Mat NN;
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

    cv::Mat ONOFF = frame.clone();
    //cv::imshow("Extracted Frame",ONOFF);/* waitKey(1) */;

    return ONOFF;
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
}


