#include "fsmodel.h"

FSModel::FSModel()
{

}

void FSModel::convertPointCloudToSurfaceMesh()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud_blob;
    pcl::io::loadPCDFile ("bearHigh.pcd", cloud_blob);
    pcl::fromROSMsg (cloud_blob, *cloud);
    //* the data should be available in cloud

    //cloud = pointCloud;

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (15.00);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    pcl::io::savePLYFile("mesh.ply", triangles);
}

void FSModel::loadPointCloud(const std::string &file_name)
{
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //mycloud.width    = 5;
    //mycloud.height   = 1;
    //mycloud.is_dense = false;
    //mycloud.points.resize (mycloud.width * mycloud.height);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, pointCloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read pcd file  \n");
        return ;
      }
      std::cout << "Loaded "
                << pointCloud.width * pointCloud.height
                << " data points from test_pcd.pcd with the following fields: "
                << std::endl;
      /*for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;*/
      FSController::getInstance()->geometries->setPointCloudTo(pointCloud);
      std::cout << "Done from Model" << std::endl;

}
