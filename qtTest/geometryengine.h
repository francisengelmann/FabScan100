#ifndef GEOMETRYENGINE_H
#define GEOMETRYENGINE_H

#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLShaderProgram>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

class GeometryEngine : protected QGLFunctions
{
public:
    GeometryEngine();
    virtual ~GeometryEngine();

    void init();

    void drawCubeGeometry(QGLShaderProgram *program);

    void initPointCloud();
    void setPointCloudTo(pcl::PointCloud<pcl::PointXYZ> pointcloud);
    void drawPointCloud(QGLShaderProgram *program);

    void initSurfaceMesh();
    void setSurfaceMeshTo(pcl::PolygonMesh surfacemesh);
    void drawSurfaceMesh(QGLShaderProgram *program);

private:
    void initCubeGeometry();
    GLuint *vboIds;
};

#endif // GEOMETRYENGINE_H
