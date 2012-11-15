#include "fsvision.h"
#include "fslaser.h"
#include "fswebcam.h"
#include "fsturntable.h"
#include "fscontroller.h"

#include <assert.h>

#include <QDebug>

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
    //cv::adaptiveThreshold(diffImage,treshImage,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,501,0);

    //cv::AdaptiveThreshold(subImage,subImage,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,501,0);
    cv::threshold(diffImage,treshImage,threshold,255,cv::THRESH_BINARY); //apply threshold

    /*cv::namedWindow("extracted laserLine");
    cv::imshow("extracted laserLine",diffImage);
    cv::waitKey(0);
    cv::imshow("extracted laserLine",treshImage);
    cv::waitKey(0);
    cvDestroyWindow("extracted laserLine");*/

    //cv::morphologyEx(treshImage,treshImage,cv::MORPH_GRADIENT,cv::Mat());
    cv::Mat element5(3,3,CV_8U,cv::Scalar(1));
    cv::morphologyEx(treshImage,treshImage,cv::MORPH_OPEN,element5);
    //cv::erode(treshImage,treshImage,cv::Mat());
    //cv::morphologyEx(treshImage,treshImage,cv::MORPH_CLOSE,element5);
    //cv::waitKey(0);
    //cv::imshow("Laser Frame",treshImage);
    //cv::waitKey(0);
    //cvDestroyWindow("Laser Frame");

    /*********************************************
     *    A LOT OF INTERESTING STUFF TO TRY!     *
     *********************************************

    cvAbsDiff(bwNoLaser,bwWithLaser,subImage);
    //cvThreshold(subImage, subImage, TRESHOLD_FOR_BW, 255, CV_THRESH_BINARY);

    //secend last param is the size of the block that is used to determine treshhold
    cvAdaptiveThreshold(subImage,subImage,255,CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY,501,0);
    cvSmooth( subImage, subImage, CV_GAUSSIAN, 11, 11 );
    cvSmooth( subImage, subImage, CV_GAUSSIAN, 11, 11 );
    cvThreshold(subImage, subImage, 200, 255, CV_THRESH_BINARY);
    */

    cv::cvtColor(treshImage, result, CV_GRAY2RGB); //convert back ro rgb
    return result;
}

cv::Mat FSVision::drawHelperLinesToFrame(cv::Mat &frame)
{
    //artifical horizont
    cv::line(frame,
             cv::Point(0,frame.rows*ORIGIN_Y),
             cv::Point(frame.cols,frame.rows*ORIGIN_Y),
             CV_RGB( 0,0,255 ),
             2);

    //two lines for center of frame
    cv::line(frame,
             cv::Point(frame.cols*0.5f,0),
             cv::Point(frame.cols*0.5f,frame.rows),
             CV_RGB( 255,255,0 ),
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

void FSVision::putPointsFromFrameToCloud(
        cv::Mat &laserOff,
        cv::Mat &laserOn,
        int dpiVertical,    //step between vertical points
        FSFloat lowerLimit, //remove points below this limit
        FSFloat threshold)  //threshold for binary images
{
    //qDebug() << "putPointsFromFrameToCloud";
    //the following lines are just to make to code more readable
    FSModel* model = FSController::getInstance()->model;
    FSLaser* laser = FSController::getInstance()->laser;
    FSTurntable* turntable = FSController::getInstance()->turntable;
    FSWebCam* webcam = FSController::getInstance()->webcam;

    //extract laser line from the two images
    cv::Mat laserLine = subLaser(laserOff,laserOn,threshold);

    //cv::namedWindow("extracted laserLine");
    //cv::imshow("extracted laserLine",laserLine);
    //cv::waitKey(0);
    //cvDestroyWindow("extracted laserLine");

    //calculate position of laser in cv frame
    FSPoint fsLaserLinePosition = laser->getLaserPointPosition();
    CvPoint cvLaserLinePosition = convertFSPointToCvPoint(fsLaserLinePosition);
    FSFloat laserPos = cvLaserLinePosition.x;

    unsigned int cols = laserLine.cols;
    unsigned int rows = laserLine.rows;
    cv::Mat bwImage( cols,rows,CV_8U,cv::Scalar(100) );
    cv::cvtColor(laserLine, bwImage, CV_RGB2GRAY); //convert to grayscale
    //now iterating from top to bottom over bwLaserLine frame
    //no bear outside of these limits :) cutting of top and bottom of frame
    for(int y = UPPER_ANALYZING_FRAME_LIMIT;
        y < bwImage.rows-LOWER_ANALYZING_FRAME_LIMIT;
        y+=dpiVertical )
    {
        //ANALYZING_LASER_OFFSET is the offset where we stop looking for a reflected laser, cos we might catch the non reflected
        //now iteratinf from right to left over bwLaserLine frame
        for(int x = bwImage.cols-1;
            x >= laserPos+ANALYZING_LASER_OFFSET;
            x -= 1){
            if(bwImage.at<uchar>(y,x)==255){
                //qDebug() << "found point at x=" << x;
                //if (row[x] > 200){
                //we have a white point in the grayscale image, so one edge laser line found
                //no we should continue to look for the other edge and then take the middle of those two points
                //to take the width of the laser line into account

                CvPoint cvNewPoint; //position of the reflected laser line on the image plane
                cvNewPoint.x = x;
                cvNewPoint.y = y;

                FSPoint fsNewPoint = FSVision::convertCvPointToFSPoint(cvNewPoint); //convert to world coordinates
                FSLine l1 = computeLineFromPoints(webcam->getPosition(), fsNewPoint);
                FSLine l2 = computeLineFromPoints(laser->getPosition(), laser->getLaserPointPosition());

                FSPoint i = computeIntersectionOfLines(l1, l2);
                fsNewPoint.x = i.x;
                fsNewPoint.z = i.z;

                //old stuff probably wrong
                //FSFloat angle = (laser->getRotation()).y;
                //fsNewPoint.z = (fsNewPoint.x - fsLaserLinePosition.x)/tan(angle*M_PI/180.0f);
                //At this point we know the depth=z. Now we need to consider the scaling depending on the depth.
                //First we move our point to a camera centered cartesion system.
                fsNewPoint.y -= (webcam->getPosition()).y;
                fsNewPoint.y *= ((webcam->getPosition()).z - fsNewPoint.z)/(webcam->getPosition()).z;
                //Redo the translation to the box centered cartesion system.
                fsNewPoint.y += (webcam->getPosition()).y;

                FSUChar r = laserOff.at<cv::Vec3b>(y,x)[2];
                FSUChar g = laserOff.at<cv::Vec3b>(y,x)[1];
                FSUChar b = laserOff.at<cv::Vec3b>(y,x)[0];
                fsNewPoint.color = FSMakeColor(r, g, b);

                //turning new point according to current angle of turntable
                //translate coordinate system to the middle of the turntable
                fsNewPoint.z -= TURNTABLE_POS_Z; //7cm radius of turntbale plus 5mm offset from back plane
                FSPoint alphaDelta = turntable->getRotation();
                FSFloat alphaOld = (float)atan(fsNewPoint.z/fsNewPoint.x);
                FSFloat alphaNew = alphaOld+alphaDelta.y*(M_PI/180.0f);
                FSFloat hypotenuse = (float)sqrt(fsNewPoint.x*fsNewPoint.x + fsNewPoint.z*fsNewPoint.z);

                if(fsNewPoint.z < 0 && fsNewPoint.x < 0){
                    alphaNew += M_PI;
                }else if(fsNewPoint.z > 0 && fsNewPoint.x < 0){
                    alphaNew -= M_PI;
                }
                fsNewPoint.z = (float)sin(alphaNew)*hypotenuse;
                fsNewPoint.x = (float)cos(alphaNew)*hypotenuse;

                if(fsNewPoint.y>lowerLimit && hypotenuse < 7){ //eliminate points from the grounds, that are not part of the model
                    //qDebug("adding point");
                    model->addPointToPointCloud(fsNewPoint);
                }
                break;
            }
        }
    }
}

FSPoint FSVision::detectLaserLine( cv::Mat &laserOff, cv::Mat &laserOn, unsigned int threshold )
{
    unsigned int cols = laserOff.cols;
    unsigned int rows = laserOff.rows;
    cv::Mat laserLine = subLaser(laserOff, laserOn, threshold);
    std::vector<cv::Vec4i> lines;
    double deltaRho = 1;
    double deltaTheta = M_PI/2;
    int minVote = 80;
    double minLength = 200;
    double maxGap = 20;
    cv::Mat laserLineBW( cols, rows, CV_8U, cv::Scalar(100) );
    cv::cvtColor(laserLine, laserLineBW, CV_RGB2GRAY); //convert to grayscale

    cv::HoughLinesP( laserLineBW,
                     lines,
                     deltaRho,
                     deltaTheta,
                     minVote,
                     minLength,
                     maxGap );

    //should at least detect the laser line
    if(lines.size()==0){
        qDebug("Did not detect any laser line, did you select a SerialPort form the menu?");
        FSPoint p = FSMakePoint(0.0,0.0,0.0);
        return(p);
    }
    //assert(lines.size()>0);
    //for(int i=0;i<lines.size();i++){
    //for(int i=0;i<1;i++){
        int i = 0;
        cv::Point p1;
        p1.x = lines[i][0];
        p1.y = lines[i][1];
        cv::Point p2;
        p2.x = lines[i][2];
        p2.y = lines[i][3];
        cv::line(laserLine, p1, p2, CV_RGB( 255,0,0 ),1);   //draw laser line
     //}



    /*cv::namedWindow("Detected Lines with HoughP");
    cv::imshow("Detected Lines with HoughP",laserLine);
    cv::waitKey(0);
    cvDestroyWindow("Detected Lines with HoughP");*/

    FSPoint p = convertCvPointToFSPoint(p1);
    return p;
}
