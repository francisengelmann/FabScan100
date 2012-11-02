#include "fscontroller.h"

FSController* FSController::singleton=0;

FSController::FSController()
{
    model = new FSModel();
    geometries = new GeometryEngine();
}

FSController* FSController::getInstance() {
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
