#-------------------------------------------------
#
# Project created by QtCreator 2012-09-17T22:38:59
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = qtTest
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    geometryengine.cpp \
    mainwidget.cpp \
    fscontroller.cpp \
    fsmodel.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    mainwidget.h \
    fscontroller.h \
    fsmodel.h

FORMS    += mainwindow.ui

OTHER_FILES += \
    fshader.glsl \
    vshader.glsl

RESOURCES += \
    shaders.qrc \
    textures.qrc

macx {
    message("Buildng for Mac.")
    INCLUDEPATH += /usr/local/Cellar/opencv/2.4.2/include

    LIBS += -LC:/usr/local/Cellar/opencv/2.4.2/ \
   -lopencv_core \
   -lopencv_highgui \
   -lopencv_imgproc \
   -lopencv_features2d \
   -lopencv_calib3d

    INCLUDEPATH += /usr/local/include/pcl-1.5
    INCLUDEPATH += /usr/local/Cellar/eigen/3.1.1/include/eigen3
    INCLUDEPATH += /usr/local/Cellar/flann/1.7.1/include

    LIBS += -LC:/usr/local/lib \
    -lpcl_common \
    -lpcl_io \
    -lpcl_filters \
    -lpcl_kdtree \
    -lpcl_registration \
    -lpcl_features \
    -lpcl_segmentation
}

linux-g++ {
    message("Buildng for Linux.")
    INCLUDEPATH += /usr/local/include/opencv2/
    INCLUDEPATH += /usr/include/eigen3/
    LIBS += -LC:/usr/local/lib/ \
   -lopencv_core \
   -lopencv_highgui \
   -lopencv_imgproc \
   -lopencv_features2d \
   -lopencv_calib3d
    #CONFIG += link_pkgconfig
    #PKGCONFIG += opencv

    INCLUDEPATH += /usr/include/pcl-1.6
    LIBS += -LC:/usr/lib \
    -lpcl_common \
    -lpcl_io \
    -lpcl_filters \
    -lpcl_kdtree \
    -lpcl_registration \
    -lpcl_features \
    -lpcl_segmentation
}

win32 {
    message("Buildng for Win. not working yet...")
}
