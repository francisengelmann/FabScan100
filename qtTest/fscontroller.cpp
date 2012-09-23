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

void FSController::loadModel() {

    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../qtTest/bearHigh.pcd", *tmpCloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
      }
      std::cout << "Loaded "
                << tmpCloud->width * tmpCloud->height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;
      for (size_t i = 0; i < tmpCloud->points.size (); ++i){
            //FSController::getInstance()->cloud;
          a=1;
      }
      //  std::cout << "    " << cloud->points[i].x
      //            << " "    << cloud->points[i].y
      //            << " "    << cloud->points[i].z << std::endl;*/
}
