#include "fsturntable.h"
#include "fscontroller.h"

FSTurntable::FSTurntable()
{
    degreesPerStep = 360.0f/200.0f/16.0f; //the size of a microstep
    direction = FS_DIRECTION_CW;
    rotation = FSMakePoint(0.0f, 0.0f, 0.0f);
}

void FSTurntable::turnNumberOfSteps(unsigned int steps)
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
    this->selectStepper();
    FSController::getInstance()->serial->writeChars(c);
}

void FSTurntable::turnNumberOfDegrees(double degrees)
{
    int steps = (int)(degrees/degreesPerStep);
    if(direction==FS_DIRECTION_CW){
      rotation.y -= degrees;
    }else if(direction==FS_DIRECTION_CCW){
      rotation.y += degrees;
    }
    turnNumberOfSteps(steps);
}

void FSTurntable::setDirection(FSDirection d)
{
    this->selectStepper();
    direction = d;
    char c = (d==FS_DIRECTION_CW)?MC_SET_DIRECTION_CW:MC_SET_DIRECTION_CCW;
    FSController::getInstance()->serial->writeChar(c);
}

void FSTurntable::toggleDirection(){
    FSDirection d = (direction == FS_DIRECTION_CW)?FS_DIRECTION_CCW:FS_DIRECTION_CW;
    setDirection(d);
}

void FSTurntable::selectStepper()
{
    char c[2];
    c[0] = MC_SELECT_STEPPER;
    c[1] = MC_TURNTABLE_STEPPER;
    FSController::getInstance()->serial->writeChars(c);
}

void FSTurntable::enable(void)
{
    this->selectStepper();
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_ON);
}

void FSTurntable::disable(void)
{
    this->selectStepper();
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_OFF);
}

void FSTurntable::setRotation(FSPoint r)
{
    rotation = r;
}

FSPoint FSTurntable::getRotation()
{
    return rotation;
}
