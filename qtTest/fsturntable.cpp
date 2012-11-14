#include "fsstepper.h"
#include "fscontroller.h"

FSStepper::FSStepper()
{
    degreesPerStep = 360.0f/200.0f/16.0f; //the size of a microstep
    direction = FS_DIRECTION_CW;
}

void FSStepper::turnNumberOfSteps(unsigned int steps)
{
    unsigned char size = steps/256*2;
    char c[size];
    unsigned int s = steps;
    for(unsigned int i=0; i<=steps/256; i++){
        c[2*i]=MC_PERFORM_STEP;
        if(s<256){
            c[2*i+1]=s%256;
        }else{
            c[2*i+1]=255;
            s-=255;
        }
    }
    FSController::getInstance()->serial->writeChars(c);
}

void FSStepper::turnNumberOfDegrees(double degrees)
{
    int steps = (int)(degrees/degreesPerStep);
    turnNumberOfSteps(steps);
}

void FSStepper::setDirection(FSDirection d)
{
    direction = d;
    char c = (d==FS_DIRECTION_CW)?MC_SET_DIRECTION_CW:MC_SET_DIRECTION_CCW;
    FSController::getInstance()->serial->writeChar(c);
}

void FSStepper::toggleDirection(){
    FSDirection d = (direction == FS_DIRECTION_CW)?FS_DIRECTION_CCW:FS_DIRECTION_CW;
    setDirection(d);
}

void FSStepper::enable(void)
{
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_ON);
}

void FSStepper::disable(void)
{
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_OFF);
}
