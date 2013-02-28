#include "fslaser.h"
#include "fscontroller.h"
#include "fsserial.h"
#include <math.h>

FSLaser::FSLaser()
{
    laserPointPosition = FSMakePoint(14.0f, 0.0f, 0.0f);
    degreesPerStep = 360.0f/200.0f/16.0f; //the size of a microstep
    direction = FS_DIRECTION_CW;
    rotation = FSMakePoint(0.0f, 0.0f, 0.0f);
    position = FSMakePoint(LASER_POS_X, LASER_POS_Y, LASER_POS_Z);
    //FSController::getInstance()->controlPanel->setLaserSwipeMaxEditText(rotation.y);
}

void FSLaser::selectStepper()
{
    char c[2];
    c[0] = MC_SELECT_STEPPER;
    c[1] = MC_LASER_STEPPER;
    FSController::getInstance()->serial->writeChars(c);
}

void FSLaser::turnOn()
{
    FSController::getInstance()->serial->writeChar(MC_TURN_LASER_ON);
}

void FSLaser::turnOff()
{
    FSController::getInstance()->serial->writeChar(MC_TURN_LASER_OFF);
}

void FSLaser::turnNumberOfSteps(unsigned int steps)
{
    this->selectStepper();
    qDebug()<<"Steps: " << steps;
    unsigned char size = steps/256*2;
    char c[size];
    unsigned int s = steps;
    for(unsigned int i=0; i<=(steps/256); i++){
        c[2*i]=MC_PERFORM_STEP;
        if(s<256){
            c[2*i+1]=s%256;
        }else{
            c[2*i+1]=255;
            s-=255;
        }
    }
    FSController::getInstance()->serial->writeChars(c);
    laserPointPosition.x = position.x - tan(rotation.y*M_PI/180)*position.z;
    qDebug() << "LaserPositionX"<< laserPointPosition.x;
}

void FSLaser::turnNumberOfDegrees(double degrees)
{
    int steps = (int)(degrees/degreesPerStep);
    //make sure to correctly update rotation in degrees, not steps
    degrees = (double)steps*(double)degreesPerStep;
    qDebug()<<"Steps"<<steps<<"Degrees"<<degrees;
    if(direction==FS_DIRECTION_CCW){
      rotation.y += degrees;
    }else if(direction==FS_DIRECTION_CW){
      rotation.y -= degrees;
    }
    qDebug()<<"computed number of steps";
    turnNumberOfSteps(steps);
    FSController::getInstance()->controlPanel->setLaserAngleText(rotation.y);
}

void FSLaser::turnToAngle(float degrees)
{
    double degreesToTurn = this->rotation.y - degrees;
    if(degreesToTurn<0){
        setDirection(FS_DIRECTION_CCW);
        turnNumberOfDegrees(degreesToTurn*-1);
    }else{
        setDirection(FS_DIRECTION_CW);
        turnNumberOfDegrees(degreesToTurn);
    }
}

void FSLaser::setDirection(FSDirection d)
{
    this->selectStepper();
    direction = d;
    char c = (d==FS_DIRECTION_CW)?MC_SET_DIRECTION_CW:MC_SET_DIRECTION_CCW;
    FSController::getInstance()->serial->writeChar(c);
}

void FSLaser::toggleDirection(){
    FSDirection d = (direction == FS_DIRECTION_CW)?FS_DIRECTION_CCW:FS_DIRECTION_CW;
    setDirection(d);
}

void FSLaser::enable(void)
{
    this->selectStepper();
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_ON);
}

void FSLaser::disable(void)
{
    this->selectStepper();
    FSController::getInstance()->serial->writeChar(MC_TURN_STEPPER_OFF);
}

void FSLaser::setLaserPointPosition(FSPoint p)
{
    laserPointPosition = p;
    double b = position.x - laserPointPosition.x;
    double a = position.z - laserPointPosition.z;
    rotation.y = atan(b/a)*180.0/M_PI;
    qDebug() << "Current laser angle: "<<rotation.y;
    FSController::getInstance()->controlPanel->setLaserAngleText(rotation.y);
}

FSPoint FSLaser::getLaserPointPosition(void)
{
    return laserPointPosition;
}

FSPoint FSLaser::getPosition()
{
    return position;
}

FSPoint FSLaser::getRotation()
{
    return rotation;
}
