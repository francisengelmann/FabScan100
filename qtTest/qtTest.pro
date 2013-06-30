#-------------------------------------------------
#
# Project created by QtCreator 2012-09-17T22:38:59
#
#-------------------------------------------------

QT += core gui opengl multimedia multimediawidgets

CONFIG += static noframework

TARGET = FabScan100
TEMPLATE = app

CONFIG += precompile_header
PRECOMPILED_HEADER = staticHeaders.h

include(qextserialport-1.2beta2/src/qextserialport.pri)

SOURCES += main.cpp\
        mainwindow.cpp \
    geometryengine.cpp \
    mainwidget.cpp \
    fscontroller.cpp \
    fsmodel.cpp \
    fsserial.cpp \
    fsdialog.cpp \
    fscontrolpanel.cpp \
    fslaser.cpp \
    fsturntable.cpp \
    fsvision.cpp \
    fswebcam.cpp \
    fsconfiguration.cpp

HEADERS  += mainwindow.h \
    geometryengine.h \
    mainwidget.h \
    fscontroller.h \
    fsmodel.h \
    staticHeaders.h \
    fsserial.h \
    fswebcam.h \
    fsdialog.h \
    fscontrolpanel.h \
    fsdefines.h \
    fsgeometries.h \
    fslaser.h \
    fsvision.h \
    fsturntable.h \
    fsconfiguration.h

FORMS    += mainwindow.ui \
    fsdialog.ui \
    fscontrolpanel.ui

OTHER_FILES += \
    fshader.glsl \
    vshader.glsl

RESOURCES += \
    shaders.qrc \
    textures.qrc

macx {
    message("Buildng for Mac.")
    INCLUDEPATH += /usr/local/Cellar/opencv/2.4.2/include
    LIBS += -L/usr/local/Cellar/opencv/2.4.2 \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_features2d \
    -lopencv_calib3d

    #LIBS += -L/usr/local/lib/vtk-5.10 \
    #-lvtkCommon \
    #-lvtkWidgets \
    #-lvtkIO \
    #-lvtkImaging \
    #-lvtkHybrid \
    #-lvtkVolumeRendering \
    #-lvtkRendering \
    #-lvtkGraphics \
    #-lvtkFiltering \
    #-lvtksys

    INCLUDEPATH += /usr/local/include/pcl-1.7
    LIBS += -L/usr/local/lib \
    -lpcl_common \
    -lpcl_io \
    -lpcl_io_ply \
    -lpcl_sample_consensus \
    -lpcl_octree \
    -lpcl_ml \
    -lpcl_filters \
    -lpcl_kdtree \
    -lpcl_registration \
    -lpcl_features \
    -lpcl_segmentation \
    -lpcl_surface \
    -lpcl_search

    #INCLUDEPATH += /usr/local/Cellar/vtk/5.10.0/include/vtk-5.10

    INCLUDEPATH += /usr/local/Cellar/eigen/3.1.2/include/eigen3
    INCLUDEPATH += /usr/local/Cellar/flann/1.7.1/include
    INCLUDEPATH += /usr/local/Cellar/boost/1.51.0/include/boost

        LIBS += -L/usr/local/Cellar/boost/1.51.0 \
        -lboost_chrono-mt \
        -lboost_context-mt \
        -lboost_date_time-mt \
        -lboost_exception-mt \
        -lboost_filesystem-mt \
        -lboost_graph-mt \
        -lboost_iostreams-mt \
        -lboost_locale-mt \
        -lboost_math_c99-mt \
        -lboost_math_c99f-mt \
        -lboost_math_c99l-mt \
        -lboost_math_tr1-mt \
        -lboost_math_tr1f-mt \
        -lboost_math_tr1l-mt \
        -lboost_prg_exec_monitor-mt \
        -lboost_program_options-mt \
        -lboost_python-mt \
        -lboost_random-mt \
        -lboost_regex-mt \
        -lboost_serialization-mt \
        -lboost_signals-mt \
        -lboost_system-mt \
        #-lboost_test_exec_monitor-mt \
        -lboost_thread-mt \
        -lboost_timer-mt \
        -lboost_unit_test_framework-mt \
        -lboost_wave-mt \
        -lboost_wserialization-mt \
}

linux-g++ {
    message("Buildng for Linux.")

    #CONFIG += link_pkgconfig
    #PKGCONFIG += opencv
    #INCLUDEPATH += /usr/local/include/opencv2/
    INCLUDEPATH += /usr/include/boost/

    LIBS += -L/usr/lib/ \
     -lboost_system \
     -lboost_filesystem \

    DEFINES += LINUX

    INCLUDEPATH += /usr/include/eigen3/
    INCLUDEPATH += /usr/include/boost/

    LIBS += -L/usr/lib/ \
     -lboost_system \
     -lboost_filesystem \

    LIBS += -L/usr/local/lib/ \
    -lopencv_core \
    -lopencv_highgui \
    -lopencv_imgproc \
    -lopencv_features2d \
    -lopencv_calib3d

    INCLUDEPATH += /usr/include/

    INCLUDEPATH += /usr/include/pcl-1.6
    LIBS += -L/usr/lib \
    -lpcl_common \
    -lpcl_io \
    -lpcl_filters \
    -lpcl_kdtree \
    -lpcl_registration \
    -lpcl_features \
    -lpcl_segmentation \
    -lpcl_surface \
    -lpcl_search
}

win32 {
    message("Buildng for Win. not working yet...")
}
