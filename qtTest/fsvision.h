#ifndef FSVISION_H
#define FSVISION_H

class FSVision
{
public:
    FSVision();
    static FSPoint convertCvPointToFSPoint(CvPoint cvPoint);
    static CvPoint convertFSPointToCvPoint(FSPoint fsPoint);

    cv::Mat subLaser(cv::Mat &frame, cv::Mat &laserFrame, FSFloat threshold);
    cv::Mat drawHelperLinesToFrame(cv::Mat &frame);
    cv::Mat drawLaserLineToFrame(cv::Mat &frame);
};

#endif // FSVISION_H
