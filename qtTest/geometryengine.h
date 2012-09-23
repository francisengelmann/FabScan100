#ifndef GEOMETRYENGINE_H
#define GEOMETRYENGINE_H

#include <QtOpenGL/QGLFunctions>
#include <QtOpenGL/QGLShaderProgram>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class GeometryEngine : protected QGLFunctions
{
public:
    GeometryEngine();
    virtual ~GeometryEngine();

    void init();

    void drawCubeGeometry(QGLShaderProgram *program);

    void initPointCloud();
    void drawPointCloud(QGLShaderProgram *program);

    void setPointCloudTo(pcl::PointCloud<pcl::PointXYZ> pointcloud);

private:
    void initCubeGeometry();

    GLuint *vboIds;

};

#endif // GEOMETRYENGINE_H
