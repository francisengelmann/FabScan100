#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "staticHeaders.h"
#include "fsmodel.h"
#include "geometryengine.h"

class GeometryEngine;
class FSModel;

class FSController
{
    private:
        static FSController* singleton;
        FSController();

    public:
        FSModel* model;
        GeometryEngine *geometries;

        //Singleton Pattern
        static FSController* getInstance();
        static void destroy();

        //Class Funtionality
};

#endif // FSCONTROLLER_H
