#include "fsvision.h"
#include "fslaser.h"
#include "fscontroller.h"

class FSLaser;
class FSController;

FSVision::FSVision()
{

}

FSPoint FSVision::convertCvPointToFSPoint(CvPoint cvPoint)
{
  CvSize cvImageSize = cvSize(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT);
  FSSize fsImageSize = FSMakeSize(FRAME_WIDTH, FRAME_WIDTH*(CAM_IMAGE_HEIGHT/CAM_IMAGE_WIDTH), 0.0f);

  //here we define the origin of the cvImage, we place it in the middle of the frame and in the corner of the two perpendiculair planes
  CvPoint origin;
  origin.x = cvImageSize.width/2.0f;
  origin.y = cvImageSize.height*ORIGIN_Y;

  FSPoint fsPoint;

  //translate
  cvPoint.x -= origin.x;
  cvPoint.y -= origin.y;
  //scale
  fsPoint.x = cvPoint.x*fsImageSize.width/cvImageSize.width;
  fsPoint.y = -cvPoint.y*fsImageSize.height/cvImageSize.height;
  fsPoint.z=0.0f;

  return fsPoint;
}

CvPoint FSVision::convertFSPointToCvPoint(FSPoint fsPoint)
{
  CvSize cvImageSize = cvSize(CAM_IMAGE_WIDTH, CAM_IMAGE_HEIGHT);
  FSSize fsImageSize = FSMakeSize(FRAME_WIDTH, FRAME_WIDTH*(CAM_IMAGE_HEIGHT/CAM_IMAGE_WIDTH), 0.0f);
  CvPoint origin;
  origin.x = cvImageSize.width/2.0f;
  origin.y = cvImageSize.height*ORIGIN_Y;

  CvPoint cvPoint;

  cvPoint.x = fsPoint.x*cvImageSize.width/fsImageSize.width;
  cvPoint.y = -fsPoint.y*cvImageSize.height/fsImageSize.height;

  //translate
  cvPoint.x += origin.x;
  cvPoint.y += origin.y;

  return cvPoint;
}

cv::Mat FSVision::subLaser(cv::Mat &laserOff, cv::Mat &laserOn, FSFloat threshold)
{
    unsigned int cols = laserOff.cols;
    unsigned int rows = laserOff.rows;
    cv::Mat bwLaserOff( cols,rows,CV_8U,cv::Scalar(100) );
    cv::Mat bwLaserOn( cols,rows,CV_8U,cv::Scalar(100) );
    cv::Mat diffImage( cols,rows,CV_8U,cv::Scalar(100) );
    cv::Mat treshImage( cols,rows,CV_8U,cv::Scalar(100) );
    cv::Mat result( cols,rows,CV_8UC3,cv::Scalar(100) );

    cv::cvtColor(laserOff, bwLaserOff, CV_RGB2GRAY); //convert to grayscale
    cv::cvtColor(laserOn, bwLaserOn, CV_RGB2GRAY); //convert to grayscale
    cv::subtract(bwLaserOn,bwLaserOff,diffImage); //subtract both grayscales
    cv::GaussianBlur(diffImage,diffImage,cv::Size(5,5),3); //gaussian filter
    cv::threshold(diffImage,treshImage,40,255,cv::THRESH_BINARY); //apply threshold

    //cv::morphologyEx(treshImage,treshImage,cv::MORPH_GRADIENT,cv::Mat());
    cv::Mat element5(3,3,CV_8U,cv::Scalar(1));
    cv::morphologyEx(treshImage,treshImage,cv::MORPH_OPEN,element5);
    //cv::erode(treshImage,treshImage,cv::Mat());
    //cv::morphologyEx(treshImage,treshImage,cv::MORPH_CLOSE,element5);
    //cv::waitKey(0);
    //cv::imshow("Laser Frame",treshImage);
    //cv::waitKey(0);
    //cvDestroyWindow("Laser Frame");

    cv::cvtColor(treshImage, result, CV_GRAY2RGB); //convert back ro rgb
    return result;
}

cv::Mat FSVision::drawHelperLinesToFrame(cv::Mat &frame)
{
    //artifical horizont
    cv::line(frame,
             cv::Point(0,frame.rows*ORIGIN_Y),
             cv::Point(frame.cols,frame.rows*ORIGIN_Y),
             CV_RGB( 0,255,0 ),
             1);

    //two lines for center of frame
    cv::line(frame,
             cv::Point(frame.cols*0.5f,0),
             cv::Point(frame.cols*0.5f,frame.rows),
             CV_RGB( 0,255,0 ),
             1);
    cv::line(frame,
             cv::Point(0,frame.rows*0.5f),
             cv::Point(frame.cols,frame.rows*0.5f),
             CV_RGB( 255,255,0 ),
             1);

    //line showing the upper limit where analyzing starts
    cv::line(frame,
             cv::Point(0,frame.rows-LOWER_ANALYZING_FRAME_LIMIT),
             cv::Point(frame.cols,frame.rows-LOWER_ANALYZING_FRAME_LIMIT),
             CV_RGB( 255,255,0 ),
             1);

    //line showing the lower limit where analyzing stops
    cv::line(frame,
             cv::Point(0,UPPER_ANALYZING_FRAME_LIMIT),
             cv::Point(frame.cols,UPPER_ANALYZING_FRAME_LIMIT),
             CV_RGB( 255,255,0 ),
             1);
    return frame;
}

cv::Mat FSVision::drawLaserLineToFrame(cv::Mat &frame)
{
    FSLaser* laser = FSController::getInstance()->laser;
    CvPoint cvLaserPoint = convertFSPointToCvPoint(laser->getLaserPointPosition());

    FSFloat vertical    = cvLaserPoint.x;
    FSFloat horizontal  = convertFSPointToCvPoint(FSMakePoint(0,0,0)).y;

    cv::Point p1 = cv::Point(vertical, 0);          //top of laser line
    cv::Point p2 = cv::Point(vertical, horizontal); //bottom of laser line
    cv::line(frame, p1, p2, CV_RGB( 255,0,0 ),6);   //draw laser line
    return frame;
}
