#ifndef FSDEFINES_H
#define FSDEFINES_H

#define MC_TURN_LASER_OFF      200
#define MC_TURN_LASER_ON       201
#define MC_PERFORM_STEP        202
#define MC_SET_DIRECTION_CW    203
#define MC_SET_DIRECTION_CCW   204
#define MC_TURN_STEPPER_ON     205
#define MC_TURN_STEPPER_OFF    206
#define MC_TURN_LIGHT_ON       207
#define MC_TURN_LIGHT_OFF      208
#define MC_ROTATE_LASER        209
#define MC_FABSCAN_PING        210
#define MC_FABSCAN_PONG        211
#define MC_SELECT_STEPPER      212
#define MC_LASER_STEPPER       11
#define MC_TURNTABLE_STEPPER   10

#define REALLY_BIG_NUMBER     99999
#define REALLY_SMALL_NUMBER  -99999

//to make the scanning process faster we ommit the lower and hight part of the cvImage
//as there is no object anyway.  The lower limit is defined by the turning table lower bound
//units are pixels, seen from the top, resp from the bottom

#define UPPER_ANALYZING_FRAME_LIMIT 0
#define LOWER_ANALYZING_FRAME_LIMIT 30

//as the actual position in the frame differs a little from calculated laserline we stop a little befor as we might catch the real non reflected laser line which creates noise
#define ANALYZING_LASER_OFFSET 90

//defining the origin in the cvFrame
//the position of intersection of back plane with ground plane in cvFrame in procent
//check the yellow laser line to calibrate, the yellow laser line should touch the bottom plane
//#define ORIGIN_Y 0.825
#define ORIGIN_Y 0.75

#define DELAY_UNTIL_CAM_SHOT 0

/********************************/
/*       CAMERA DEFINES         */
/********************************/

//logitech c270
#define FRAME_WIDTH 26.6f //in cm. the width of what the camera sees, ie place a measure tool at the back-plane and see how many cm the camera sees.
#define CAM_IMAGE_WIDTH 1280.0f
#define CAM_IMAGE_HEIGHT 960.0f //here I am not sure, i think it is 960, (old=853.0f)

/********************************/
/*    HARDWARE SETUP DEFINES    */
/********************************/

//position of the laser
#define LASER_POS_X 14.0f //precise by construction
#define LASER_POS_Y 6.4f  //not needed/used for calculations
#define LASER_POS_Z 28.8f //precise by construction

#define LASER_SWIPE_MIN 18.0f
#define LASER_SWIPE_MAX 52.0f

//position of the c270
#define CAM_POS_X 0.0f //precise by construction
//#define CAM_POS_Y 4.07f
#define CAM_POS_Y 5.57f
#define CAM_POS_Z 30.9f

//position of the turntable
#define TURNTABLE_POS_X 0.0f //not used by calculations
#define TURNTABLE_POS_Y 0.0f //not used by calculations
#define TURNTABLE_POS_Z 7.5f //precise by construction

#endif // FSDEFINES_H

