/**
 * This class controls the webcam and provides functionality
 * such as listing all available cams or grabbing images.
 */

#ifndef FSWEBCAM_H
#define FSWEBCAM_H

struct {
    QString portName;
    QString friendlyName;
} FSWebCamInfo;

class FSWebCam
{
public:
    FSWebCam();
    ~FSWebCam();

    QList<FSWebCamInfo>

};

#endif // FSWEBCAM_H
