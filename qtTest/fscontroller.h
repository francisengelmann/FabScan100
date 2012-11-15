#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "staticHeaders.h"
#include "mainwindow.h"
#include "geometryengine.h"
#include "fsmodel.h"
#include "fsserial.h"
#include "fswebcam.h"
#include "fsturntable.h"
#include "fslaser.h"
#include "fsvision.h"

class GeometryEngine;
class FSModel;
/*class FSSerial;
class FSWebCam;
class MainWindow;
class FSLaser;*/

class FSController
{
    private:
        static FSController* singleton;
        FSController();

    public:
        MainWindow* mainwindow;
        GeometryEngine* geometries;
        FSModel* model;
        FSSerial* serial;
        FSWebCam* webcam;
        FSTurntable* turntable;
        FSLaser* laser;
        FSVision* vision;

        //Singleton Pattern
        static FSController* getInstance();
        static void destroy();

        //FabScan Functionality
        void fetchFrame();
        void hideFrame();

        void scan();
        void detectLaserLine();
        void computeSurfaceMesh();
};

#endif // FSCONTROLLER_H
