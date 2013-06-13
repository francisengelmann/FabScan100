#ifndef GEOMETRYENGINE_H
#define GEOMETRYENGINE_H

#include "staticHeaders.h"

class GeometryEngine : protected QGLFunctions
{
public:
    GeometryEngine();
    virtual ~GeometryEngine();

    void init();

    void drawCubeGeometry(QGLShaderProgram *program);

    void initPointCloud();
    void setPointCloudTo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);
    void drawPointCloud(QGLShaderProgram *program);

    void initSurfaceMesh();
    void setSurfaceMeshTo(pcl::PolygonMesh &surfacemesh, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointCloud);
    void drawSurfaceMesh(QGLShaderProgram *program);

    void initGroundPlane();
    void drawGroundPlane(QGLShaderProgram *program);

private:
    void initCubeGeometry();
    GLuint *vboIds;
    GLuint numberOfIndices;
};

#endif // GEOMETRYENGINE_H
