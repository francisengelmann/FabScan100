#include "fslaser.h"
#include "fscontroller.h"
#include "fsserial.h"

FSLaser::FSLaser()
{
    laserPointPosition = FSMakePoint(-4.0f, 0.0f, 0.0f);
}

void FSLaser::turnOn()
{
    FSController::getInstance()->serial->writeChar(MC_TURN_LASER_ON);
}

void FSLaser::turnOff()
{
    FSController::getInstance()->serial->writeChar(MC_TURN_LASER_OFF);
}

void FSLaser::setLaserPointPosition(FSPoint p)
{
    laserPointPosition = p;
}

FSPoint FSLaser::getLaserPointPosition(void)
{
    return laserPointPosition;
}

FSPoint FSLaser::getPosition()
{
    return FSMakePoint(LASER_POS_X,LASER_POS_Y,LASER_POS_Z);
}
