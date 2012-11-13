#ifndef FSCONTROLLER_H
#define FSCONTROLLER_H

#include "staticHeaders.h"
#include "fsmodel.h"
#include "geometryengine.h"
#include "fsserial.h"
#include "fswebcam.h"
#include "mainwindow.h"

class GeometryEngine;
class FSModel;
class FSSerial;
class FSWebCam;
class MainWindow;

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
        MainWindow* mainwindow;

        //Singleton Pattern
        static FSController* getInstance();
        static void destroy();

        //FabScan Functionality
        void fetchFrame();
        void hideFrame();

        void turnLaserOn();
        void turnLaserOff();
        void detectLaserLine();

        void turnStepperOn();
        void turnStepperOff();
        void performStep();
        void performSteps(unsigned char steps);
        void setDirectionCW();
        void setDirectionCCW();
};

#endif // FSCONTROLLER_H
