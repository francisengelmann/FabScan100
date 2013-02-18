#ifndef FSMODEL_H
#define FSMODEL_H

#include "staticHeaders.h"
#include "fscontroller.h"
#include "geometryengine.h"

class FSModel
{
    private:

    public:
        FSModel();
        void convertPointCloudToSurfaceMesh();
        void convertPointCloudToSurfaceMesh2();
        void loadPointCloudFromPLY(const std::string &file_name);
        void loadPointCloudFromPCD(const std::string &file_name);
        void savePointCloudAsPLY(const std::string &file_name);
        void savePointCloudAsPCD(const std::string &file_name);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPoisson;
        pcl::PolygonMesh surfaceMesh;
        pcl::PolygonMesh surfaceMeshPoisson;
        void addPointToPointCloud(FSPoint point);
        // make sure to add these macro
        // see http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // FSMODEL_H
