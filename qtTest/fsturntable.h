#ifndef FSSTEPPER_H
#define FSSTEPPER_H

#include "staticHeaders.h"
#include "fsserial.h"

class FSStepper
{
private:
    double degreesPerStep;
    double direction;

public:
    FSStepper();
    void turnNumberOfSteps(unsigned int steps);
    void turnNumberOfDegrees(double degrees);
    void setDirection(FSDirection direction);
    void toggleDirection();
    void enable(void);
    void disable(void);
};

#endif // FSSTEPPER_H
