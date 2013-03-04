#include "fsmodel.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

FSModel::FSModel()
{
    pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void FSModel::convertPointCloudToSurfaceMesh()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //sensor_msgs::PointCloud2 cloud_blob;
    //pcl::io::loadPCDFile ("bearHigh.pcd", cloud_blob);
    //pcl::fromROSMsg (cloud_blob, *cloud);
    //* the data should be available in cloud

    cloud->points.resize(pointCloud->size());
    for (size_t i = 0; i < pointCloud->points.size(); i++) {
        cloud->points[i].x = pointCloud->points[i].x;
        cloud->points[i].y = pointCloud->points[i].y;
        cloud->points[i].z = pointCloud->points[i].z;
    }

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setRadiusSearch(15.0);
    //n.setKSearch (30);
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
    gp3.reconstruct (surfaceMesh);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    //pcl::io::savePLYFile("mesh.ply", surfaceMesh);
    FSController::getInstance()->meshComputed=true;
}

void FSModel::convertPointCloudToSurfaceMesh2()
{
    /*pcl::MovingLeastSquares<PointXYZRGB, PointXYZ> mls;
    mls.setInputCloud (pointCloud);
    mls.setSearchRadius (0.01);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointXYZRGB, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);

    pcl::PointCloud<PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<PointXYZ> ());
    mls.process (*cloud_smoothed);*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(pointCloud->size());
    for (size_t i = 0; i < pointCloud->points.size(); i++) {
        cloud->points[i].x = pointCloud->points[i].x;
        cloud->points[i].y = pointCloud->points[i].y;
        cloud->points[i].z = pointCloud->points[i].z;
    }

    pcl::NormalEstimation<pcl::PointXYZ, Normal> ne;
    //ne.setNumberOfThreads(8);
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    qDebug() << "Centroid:"<<centroid[0] <<centroid[1]<<centroid[2];
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
    ne.compute (*cloud_normals);
    //invert all normals, primarly they all point to the centroid
    for (size_t i = 0; i < cloud_normals->size (); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
    concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);
    //concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);*/

    Poisson<PointNormal> poisson;
    poisson.setDepth (6);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(surfaceMeshPoisson);
    //pcl::io::savePLYFile("meshPoisson.ply", surfaceMeshPoisson);
    FSController::getInstance()->meshComputed=true;
}

void FSModel::loadPointCloudFromPCD(const std::string &file_name)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_name, *pointCloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file  \n");
        return ;
    }
    std::cout   << "Loaded "
                << pointCloud->width * pointCloud->height
                << " data points from: "<< file_name
                << std::endl;
    FSController::getInstance()->geometries->setPointCloudTo(pointCloud);
    std::cout << "Done from Model" << std::endl;
}

void FSModel::loadPointCloudFromPLY(const std::string &file_name)
{
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (file_name, *pointCloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read ply file  \n");
        return ;
    }
    std::cout   << "Loaded "
                << pointCloud->width * pointCloud->height
                << " data points from: "<< file_name
                << std::endl;
    FSController::getInstance()->geometries->setPointCloudTo(pointCloud);
    std::cout << "Done from Model" << std::endl;
}

void FSModel::savePointCloudAsPCD(const std::string &file_name)
{
    if(pointCloud->size()==0) return;
    pcl::io::savePCDFileASCII (file_name, *pointCloud);
    std::cerr << "Saved " << pointCloud->points.size() << " data points to " << file_name << std::endl;
}

void FSModel::savePointCloudAsPLY(const std::string &file_name)
{
    if(pointCloud->size()==0) return;
    pcl::io::savePLYFileASCII (file_name, *pointCloud);
    std::cerr << "Saved " << pointCloud->points.size() << " data points to " << file_name << std::endl;
}

void FSModel::addPointToPointCloud(FSPoint point)
{
    //qDebug()<<"added Point to cloud";
    pcl::PointXYZRGB p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    int rgb = ((int)point.color.red) << 16 | ((int)point.color.green) << 8 | ((int)point.color.blue);
    p.rgb = rgb;
    pointCloud->push_back(p);
}
