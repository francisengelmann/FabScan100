#ifndef FSSTEPPER_H
#define FSSTEPPER_H

#include "staticHeaders.h"
#include "fsserial.h"

class FSTurntable
{
private:
    double direction; //in which direction is the table turning, left or right
    FSPoint rotation; //the current rotation of the turntable

public:
    double degreesPerStep; //the stepper performs step of a certain number of degrees

    FSTurntable();
    void turnNumberOfSteps(unsigned int steps); //tell turntable to move a certain number of steps
    void turnNumberOfDegrees(double degrees);   //tell turntable to move a certain number of degrees
    void setDirection(FSDirection direction);   //set the direction of the turntable, either clockwise or counterclock wise
    void toggleDirection();                     //change the direction
    void setRotation(FSPoint r);                //set the current rotation of the turntable
    FSPoint getRotation(void);                  //get the current rotation of the turntable
    void selectStepper();
    void enable(void);                          //enable the turntable (enable the stepper engine)
    void disable(void);                         //disable the turntable (disable the stepper engine)
};

#endif // FSSTEPPER_H
