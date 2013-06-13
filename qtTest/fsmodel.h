#ifndef FSMODEL_H
#define FSMODEL_H

#include "staticHeaders.h"
#include "fscontroller.h"
#include "geometryengine.h"
#include <vector>

using namespace std;

class FSModel
{
    private:

    public:
        FSModel();
        void convertPointCloudToSurfaceMesh();
        void convertPointCloudToSurfaceMesh2();
        void convertPointCloudToSurfaceMesh3();
        void loadPointCloudFromPLY(const std::string &file_name);
        void loadPointCloudFromPCD(const std::string &file_name);
        void savePointCloudAsPLY(const std::string &file_name);
        void savePointCloudAsPCD(const std::string &file_name);
        void savePointCloudAsPTS(const std::string &file_name);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPoisson;
        pcl::PolygonMesh surfaceMesh;
        pcl::PolygonMesh surfaceMeshPoisson;
        void addPointToPointCloud(FSPoint point);

        void loadSurfaceMeshFromOFF(const std::string &file_name);

        unsigned int openFromOFFFile(const char* offFilePath);
        unsigned int convertPolygons2Triangles(void);
        unsigned int saveToSTLFile(string stlFilePath);

        vector<FSPoint> vertexVector;               // contains the points/vertices from a .off file [v_1][v_2][v_3]...
        vector<vector <unsigned int> > faceVector;  // contains the indices to define faces [[v_a][v_b]...] [[v_c][v_d]...]...

        // make sure to add these macro
        // see http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // FSMODEL_H
