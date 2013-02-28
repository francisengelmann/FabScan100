#ifndef FSLASER_H
#define FSLASER_H

#include "staticHeaders.h"

class FSLaser
{
private:
    FSPoint laserPointPosition; //where the laser points on the backplane
    FSPoint rotation; //the current rotation of the turntable
    double direction; //in which direction is the table turning, left or right
    FSPoint position;

public:
    double degreesPerStep; //the stepper performs step of a certain number of degrees
    FSLaser();

    void turnOn();
    void turnOff();

    void enable();
    void disable();

    void selectStepper();

    void turnNumberOfSteps(unsigned int steps); //tell turntable to move a certain number of steps
    void turnNumberOfDegrees(double degrees);   //tell turntable to move a certain number of degrees
    void turnToAngle(float degrees);

    void setDirection(FSDirection direction);   //set the direction of the turntable, either clockwise or counterclock wise
    void toggleDirection();                     //change the direction

    void setRotation(FSPoint r);                //set the current rotation of the turntable
    FSPoint getRotation(void);                  //get the current rotation of the turntable

    void setLaserPointPosition(FSPoint p);
    FSPoint getLaserPointPosition(void);

    FSPoint getPosition(void);

};

#endif // FSLASER_H
