#include "fscontroller.h"

FSController* FSController::singleton=0;
//FSController cloud = new pcl::PointCloud<pcl::PointXZY>

FSController::FSController()
{
    //create all the stuff
    //cdetect= new ColorDetector();
    //cloud = new pcl::PointCloud<pcl::PointXYZ>;
    model = new FSModel();
    geometries = new GeometryEngine();
}

FSController* FSController::getInstance() {
    // Creates the instance at first call
    if (singleton == 0){
        singleton = new FSController();
    }

    return singleton;
}

void FSController::destroy() {
    if (singleton != 0) {
        delete singleton;
        singleton = 0;
    }
}
