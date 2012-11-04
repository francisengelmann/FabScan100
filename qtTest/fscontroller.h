#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "staticHeaders.h"
#include "fsmodel.h"
#include "geometryengine.h"
#include "fsserial.h"

class GeometryEngine;
class FSModel;
class FSSerial;

class FSController
{
    private:
        static FSController* singleton;
        FSController();

    public:
        FSModel* model;
        GeometryEngine* geometries;
        FSSerial* serial;

        //Singleton Pattern
        static FSController* getInstance();
        static void destroy();

        //Class Funtionality
};

#endif // FSCONTROLLER_H
