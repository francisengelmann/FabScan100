#-------------------------------------------------
#
# Project created by QtCreator 2012-09-17T22:38:59
#
#-------------------------------------------------

QT       += core gui opengl opencv

TARGET = qtTest
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    geometryengine.cpp \
    mainwidget.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    mainwidget.h

FORMS    += mainwindow.ui

OTHER_FILES += \
    fshader.glsl \
    vshader.glsl

RESOURCES += \
    shaders.qrc \
    textures.qrc

INCLUDEPATH += /usr/local/Cellar/opencv/2.4.2/include

LIBS += -LC:/usr/local/Cellar/opencv/2.4.2/ \
   -lopencv_core \
   -lopencv_highgui \
   -lopencv_imgproc \
   -lopencv_features2d \
   -lopencv_calib3d
