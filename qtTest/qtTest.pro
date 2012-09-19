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
    mainwidget.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    mainwidget.h

FORMS    += mainwindow.ui

OTHER_FILES += \
    fshader.glsl \
    vshader.glsl

RESOURCES += \
    shaders.qrc
