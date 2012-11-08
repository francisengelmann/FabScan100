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

        //Class Funtionality
};

#endif // FSCONTROLLER_H
