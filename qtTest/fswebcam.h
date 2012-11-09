/**
 * This class controls the webcam and provides functionality
 * such as listing all available cams or grabbing images.
 */

#ifndef FSWEBCAM_H
#define FSWEBCAM_H

struct FSWebCamInfo{
    QString portName;       //path to the webcam e.g. /dev/video0
    QString friendlyName;   //name of webcam that is displayed
};

class FSWebCam
{
public:
    FSWebCam();
    ~FSWebCam();

    cv::Mat getFrame();

    static QList<FSWebCamInfo> enumerate();
};

#endif // FSWEBCAM_H
