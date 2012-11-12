#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include "staticHeaders.h"

QT_BEGIN_NAMESPACE
class QBasicTimer;
class QGLShaderProgram;
QT_END_NAMESPACE

class GeometryEngine;

class MainWidget : public QGLWidget, protected QGLFunctions
{
    Q_OBJECT
public:
    explicit MainWidget(QWidget *parent = 0);
    virtual ~MainWidget();
    char drawState;
    void paintGL();

signals:

public slots:

protected:
    void wheelEvent(QWheelEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void timerEvent(QTimerEvent *e);

    void initializeGL();
    void resizeGL(int w, int h);

    void initShaders();
    void initTextures();

private:
    QBasicTimer *timer;
    QGLShaderProgram *program;
    GeometryEngine *geometries;
    GLuint texture;

    //Transformation matrices for the camera
    QMatrix4x4 projection;
    QVector3D eye;
    QVector3D center;
    QVector3D up;
    double angleX;
    double angleY;
    double angleXtmp;
    double angleYtmp;
    double distance;

    QVector2D mousePressPosition;
    QVector3D rotationAxis;
    qreal angularSpeed;
    QQuaternion rotation;
};

#endif // MAINWIDGET_H
