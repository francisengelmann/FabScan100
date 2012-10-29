#ifndef FSMODEL_H
#define FSMODEL_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "fscontroller.h"
#include "geometryengine.h"

class FSModel
{
    private:

    public:
        FSModel();
        void convertPointCloudToSurfaceMesh();
        void loadPointCloud(const std::string &file_name);
        pcl::PointCloud<pcl::PointXYZ> pointCloud;
        pcl::PolygonMesh triangles;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure to add these macro, see http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
};

#endif // FSMODEL_H
