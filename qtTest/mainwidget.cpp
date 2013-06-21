#include "mainwidget.h"

#include "fscontroller.h"
#include "geometryengine.h"

#include <QtOpenGL/QGLShaderProgram>

#include <QBasicTimer>
#include <QMouseEvent>
#include <QDebug>

#include <math.h>
#include <locale.h>

MainWidget::MainWidget(QWidget *parent) :
    QGLWidget(parent),
    timer(new QBasicTimer),
    program(new QGLShaderProgram)
{
    distance = 15;
    angleX = (float)M_PI/2.0f;
    angleY = 1;

    angleXtmp = angleX;
    angleYtmp = angleY;

    eye =     QVector3D(0.0, 5.0, 10.0);
    center =  QVector3D(0.0, 5.0, 0.0);
    up =      QVector3D(0.0, 1.0, 0.0);
    this->setStyleSheet( "border: 2px solid black;  background-color: rgb(0, 170, 255); }" );
}

MainWidget::~MainWidget()
{
    delete timer; timer = 0;
    delete program; program = 0;
    deleteTexture(texture);
}

void MainWidget::mousePressEvent(QMouseEvent *e)
{
    // Saving mouse press position
    //mousePressPosition = QVector2D(e->posF());
    mousePressPosition = QVector2D(e->localPos());
    FSController::getInstance()->mainwindow->setCursor(Qt::ClosedHandCursor);
}

void MainWidget::wheelEvent(QWheelEvent *e){
    distance -= e->delta()/300.0f;
    if(distance<1.0){
        distance=1.0;
    }else if(distance>30.0){
        distance=30.0;
    }
    this->updateGL();
}

void MainWidget::mouseMoveEvent(QMouseEvent *e){
    //QVector2D diff = QVector2D(e->posF()) - mousePressPosition;
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    double newAngleX = angleX-diff.x()/300.0f;
    if(newAngleX>M_PI ){ newAngleX-=2*(float)M_PI;}
    if(newAngleX<-M_PI){ newAngleX+=2*(float)M_PI;}

    double newAngleY = angleY-diff.y()/300.0f;
    if(newAngleY>M_PI-0.01){ newAngleY=(float)(M_PI-0.01);}
    if(newAngleY<0.01){ newAngleY=(float)0.01; }
    angleXtmp=newAngleX;
    angleYtmp=newAngleY;


    this->updateGL();
    //qDebug() << diff.lengthSquared();
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e)
{
    angleX=angleXtmp;
    angleY=angleYtmp;
    FSController::getInstance()->mainwindow->setCursor(Qt::OpenHandCursor);
}

void MainWidget::timerEvent(QTimerEvent *e)
{
    Q_UNUSED(e);

    // Decrease angular speed (friction)
    angularSpeed *= 0.99;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01)
        angularSpeed = 0.0;
    else {
        // Update rotation
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        // Update scene
        this->updateGL();
    }
}

void MainWidget::initializeGL()
{
    drawState = 0;

    initializeGLFunctions();

    qglClearColor(Qt::white);

    qDebug() << "Initializing shaders...";
    initShaders();

    qDebug() << "Initializing textures...";
    initTextures();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    //glEnable(GL_CULL_FACE);


    qDebug() << "Initializing geometries...";
    FSController::getInstance()->geometries->init();

    // using QBasicTimer because its faster that QTimer
    timer->start(12, this);
}

void MainWidget::initShaders()
{
    // Overriding system locale until shaders are compiled
    setlocale(LC_NUMERIC, "C");

    // Compiling vertex shader
    if (!program->addShaderFromSourceFile(QGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compiling fragment shader
    if (!program->addShaderFromSourceFile(QGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Linking shader pipeline
    if (!program->link())
        close();

    // Binding shader pipeline for use
    if (!program->bind())
        close();

    // Restore system locale
    setlocale(LC_ALL, "");
}

void MainWidget::initTextures()
{
    // Loading cube.png to texture unit 0
    glActiveTexture(GL_TEXTURE0);
    glEnable(GL_TEXTURE_2D);
    texture = bindTexture(QImage(":/cube.png"));

    // Set nearest filtering mode for texture minification
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Set bilinear filtering mode for texture magnification
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

void MainWidget::resizeGL(int w, int h)
{
    // Set OpenGL viewport to cover whole widget
    glViewport(0, 0, w, h);

    // Calculate aspect ratio
    qreal aspect = (qreal)w / ((qreal)h?h:1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 1.0, zFar = 40.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}

void MainWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 view;
    view.setToIdentity();
    QMatrix4x4 model;
    model.setToIdentity();

    // Calculate model view transformation
    //model.rotate(rotation);


    eye.setX(-distance*sin(angleYtmp)*cos(angleXtmp)+center.x() );
    eye.setY( distance*cos(angleYtmp)+center.y() );
    eye.setZ( distance*sin(angleYtmp)*sin(angleXtmp)+center.z() );

    view.lookAt(eye, center, up);

    //matrix.translate(0.0, -5.0, -15.0);
    //matrix.translate(0.0, -5.0, 0.0);

    // Set modelview-projection matrix
    program->setUniformValue("mvp_matrix", projection * view * model);

    // Using texture unit 0 which contains cube.png
    program->setUniformValue("texture", 0);

    // Draw cube geometry
    //geometries->drawCubeGeometry(program);

    if(drawState == 0){//POINT_CLOUD
        FSController::getInstance()->geometries->drawPointCloud(program);
    }else if(drawState == 1){//SURFACE_MESH
        FSController::getInstance()->geometries->drawSurfaceMesh(program);
    }
    FSController::getInstance()->geometries->drawGroundPlane(program);
}
