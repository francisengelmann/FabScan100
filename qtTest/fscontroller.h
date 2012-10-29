#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include <QtOpenGL/QGLWidget>
#include <QtOpenGL/QGLFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
