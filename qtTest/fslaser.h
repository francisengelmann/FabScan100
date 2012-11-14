#ifndef FSLASER_H
#define FSLASER_H

#include "staticHeaders.h"

class FSLaser
{
private:
    FSPoint laserPointPosition; //where the laser points on the backplane

public:
    FSLaser();
    void turnOn();
    void turnOff();

    void setLaserPointPosition(FSPoint p);
    FSPoint getLaserPointPosition(void);

};

#endif // FSLASER_H
