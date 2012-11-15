#ifndef FSSTEPPER_H
#define FSSTEPPER_H

#include "staticHeaders.h"
#include "fsserial.h"

class FSTurntable
{
private:
    double direction;
    FSPoint rotation;

public:
    double degreesPerStep;

    FSTurntable();
    void turnNumberOfSteps(unsigned int steps);
    void turnNumberOfDegrees(double degrees);
    void setDirection(FSDirection direction);
    void toggleDirection();
    void setRotation(FSPoint r);
    FSPoint getRotation(void);
    void enable(void);
    void disable(void);
};

#endif // FSSTEPPER_H
