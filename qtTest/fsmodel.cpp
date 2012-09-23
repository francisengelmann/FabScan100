#include "fsmodel.h"

FSModel::FSModel()
{

}

void FSModel::loadModel()
{
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //mycloud.width    = 5;
    //mycloud.height   = 1;
    //mycloud.is_dense = false;
    //mycloud.points.resize (mycloud.width * mycloud.height);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../qtTest/bearHigh.pcd", mycloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return ;
      }
      std::cout << "Loaded "
                << mycloud.width * mycloud.height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;
      /*for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;*/
      FSController::getInstance()->geometries->setPointCloudTo(mycloud);
      std::cout << "Done from Model" << std::endl;

}
