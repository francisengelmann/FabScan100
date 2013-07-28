#ifndef FSCONFIGURATION_H
#define FSCONFIGURATION_H

#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class FSConfiguration
{
public:
    //as the actual position in the frame differs a little from calculated laserline we stop a little befor as we might catch the real non reflected laser line which creates noise
    int ANALYZING_LASER_OFFSET;

    //to make the scanning process faster we ommit the lower and hight part of the cvImage
    //as there is no object anyway.  The lower limit is defined by the turning table lower bound
    //units are pixels, seen from the top, resp from the bottom
    int UPPER_ANALYZING_FRAME_LIMIT;
    int LOWER_ANALYZING_FRAME_LIMIT;

    //defining the origin in the cvFrame
    //the position of intersection of back plane with ground plane in cvFrame in percent
    //check the yellow laser line to calibrate, the yellow laser line should touch the bottom plane
    //#define ORIGIN_Y 0.825
    float ORIGIN_Y;

    //logitech c270
    float FRAME_WIDTH; //in cm. the width of what the camera sees, ie place a measure tool at the back-plane and see how many cm the camera sees.
    int CAM_IMAGE_WIDTH;
    int CAM_IMAGE_HEIGHT; //here I am not sure, i think it is 960, (old=853.0f)

    //position of the laser
    float LASER_POS_X; //precise by construction
    float LASER_POS_Y; //not needed/used for calculations
    float LASER_POS_Z; //precise by construction

    float LASER_SWIPE_MIN;
    float LASER_SWIPE_MAX;

    //position of the c270
    float CAM_POS_X; //precise by construction
    float CAM_POS_Y;
    float CAM_POS_Z;

    //position of the turntable
    float TURNTABLE_POS_X; //not used by calculations
    float TURNTABLE_POS_Y; //not used by calculations
    float TURNTABLE_POS_Z; //precise by construction

    int flag;

public:
    bool goodInput;
    FSConfiguration();
    void setDefaultValues();

    void read(const FileNode& node);                          //Read serialization for this class
    bool readConfiguration();
    void interprate();
};

static void read(const FileNode& node, FSConfiguration& x, const FSConfiguration& default_value = FSConfiguration())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

#endif // FSCONFIGURATION_H
