#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "staticHeaders.h"
#include "fsmodel.h"
#include "geometryengine.h"
#include "fsserial.h"
#include "fswebcam.h"

class GeometryEngine;
class FSModel;
class FSSerial;
class FSWebCam;

class FSController
{
    private:
        static FSController* singleton;
        FSController();

    public:
        FSModel* model;
        GeometryEngine* geometries;
        FSSerial* serial;
        FSWebCam* webcam;

        //Singleton Pattern
        static FSController* getInstance();
        static void destroy();

        void fetchFrame();
        void hideFrame();

        void turnLaserOn();
        void turnLaserOff();
        void detectLaserLine();

        void turnStepperOn();
        void turnStepperOff();
        void performStep();
        void setDirectionCW();
        void setDirectionCCW();
};

#endif // FSCONTROLLER_H
