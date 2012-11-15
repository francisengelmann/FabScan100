#ifndef FSVISION_H
#define FSVISION_H

class FSVision
{
public:
    FSVision();
    static FSPoint convertCvPointToFSPoint(CvPoint cvPoint);
    static CvPoint convertFSPointToCvPoint(FSPoint fsPoint);

    cv::Mat subLaser(cv::Mat &laserOff, cv::Mat &laserOn, FSFloat threshold);
    cv::Mat drawHelperLinesToFrame(cv::Mat &frame);
    cv::Mat drawLaserLineToFrame(cv::Mat &frame);

    void putPointsFromFrameToCloud(
            cv::Mat &laserOff,
            cv::Mat &laserOn,
            int dpiVertical,
            FSFloat lowerLimit,
            FSFloat threshold);

    FSPoint detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn, unsigned int threshold);
};

#endif // FSVISION_H
