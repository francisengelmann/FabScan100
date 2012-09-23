#ifndef FSMODEL_H
#define FSMODEL_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "fscontroller.h"
#include "geometryengine.h"

class FSModel
{
    private:

    public:
        FSModel();
        void loadModel();
        pcl::PointCloud<pcl::PointXYZ> mycloud;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure to add these macro, see http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
};

#endif // FSMODEL_H
