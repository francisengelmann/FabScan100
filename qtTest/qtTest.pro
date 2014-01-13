#-------------------------------------------------
#
# Project created by QtCreator 2012-09-17T22:38:59
#
#-------------------------------------------------

QT += core gui opengl multimedia multimediawidgets

CONFIG += static noframework console

#QMAKE_LFLAGS += -static

TARGET = FabScan100
TEMPLATE = app

CONFIG += precompile_header
PRECOMPILED_HEADER = staticHeaders.h

include(qextserialport-1.2rc/src/qextserialport.pri)

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
    fsconfiguration.cpp \
    fswebcam_win.cpp \
    fswebcam_unix.cpp

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
    fsconfiguration.h \
    fswebcam_win.h \
    fswebcam_unix.h

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

    QMAKE_MAC_SDK.macosx.path = /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX10.8.sdK
    QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.8

    if( !exists( $$QMAKE_MAC_SDK.macosx.path) ) {
        error("The selected Mac OSX SDK does not exist at $$QMAKE_MAC_SDK.macosx.path!")
    }
    else
    {
        message("The selected Mac OSX SDK was found at $$QMAKE_MAC_SDK.macosx.path");
        message("building for Mac OSX $$QMAKE_MACOSX_DEPLOYMENT_TARGET");
    }


    INCLUDEPATH += /usr/local/include/
    INCLUDEPATH += /usr/local/include/opencv/
    LIBS += -L/usr/local/lib/ \
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

    INCLUDEPATH += /usr/local/include/eigen3
    INCLUDEPATH += /usr/local/include/flann
    INCLUDEPATH += /usr/local/include/boost

        LIBS += -L/usr/local/lib \
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

    DEFINES += MACOSX
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
    message("Buildng for Win.")

    #fix LNK1123
    QMAKE_LFLAGS += /INCREMENTAL:NO


    #CHECK IF THESE PATHS MATCH YOUR SYSTEM !!!
    OPENCVDIR = "F:\libs\opencv-2.4.7\build\x86\vc10"
    PCLDIR = "F:\libs\pcl-1.6.0"

    exists($$OPENCVDIR) {
        DEFINES += USEOPENCV
        INCLUDEPATH += F:\libs\opencv-2.4.7\build\include

        CONFIG(release, debug|release) {
        LIBS += -L$${OPENCVDIR}/lib \
        -lopencv_core247 \
        -lopencv_highgui247 \
        -lopencv_imgproc247 \
        -lopencv_features2d247 \
        -lopencv_calib3d247 \
        -lopencv_flann247

        PRE_TARGETDEPS += \
        $${OPENCVDIR}/lib/opencv_core247.lib \
        $${OPENCVDIR}/lib/opencv_highgui247.lib \
        $${OPENCVDIR}/lib/opencv_imgproc247.lib \
        $${OPENCVDIR}/lib/opencv_features2d247.lib \
        $${OPENCVDIR}/lib/opencv_calib3d247.lib \
        $${OPENCVDIR}/lib/opencv_flann247.lib
        }else{
        LIBS += -L$${OPENCVDIR}/lib \
        -lopencv_core247d \
        -lopencv_highgui247d \
        -lopencv_imgproc247d \
        -lopencv_features2d247d \
        -lopencv_calib3d247d \
        -lopencv_flann247d

        PRE_TARGETDEPS += \
        $${OPENCVDIR}/lib/opencv_core247d.lib \
        $${OPENCVDIR}/lib/opencv_highgui247d.lib \
        $${OPENCVDIR}/lib/opencv_imgproc247d.lib \
        $${OPENCVDIR}/lib/opencv_features2d247d.lib \
        $${OPENCVDIR}/lib/opencv_calib3d247d.lib \
        $${OPENCVDIR}/lib/opencv_flann247d.lib
        }
        message("OpenCV libraries found in $${OPENCVDIR}")
      } else {
        message("OpenCV libraries not found.")
      }

    exists($$PCLDIR) {
        INCLUDEPATH += $${PCLDIR}/include/pcl-1.6
        INCLUDEPATH += $${PCLDIR}/3rdParty/Eigen/include
        INCLUDEPATH += $${PCLDIR}/3rdParty/FLANN/include
        INCLUDEPATH += $${PCLDIR}/3rdParty/Boost/include

        CONFIG(release, debug|release) {
        LIBS += -L$${PCLDIR}/lib \
        -lpcl_common_release \
        -lpcl_io_release \
        -lpcl_filters_release \
        -lpcl_kdtree_release \
        -lpcl_registration_release \
        -lpcl_features_release \
        -lpcl_segmentation_release \
        -lpcl_surface_release \
        -lpcl_search_release \

        LIBS += -L$${PCLDIR}/3rdParty/FLANN/lib \
        -lflann_s

        LIBS += -L$${PCLDIR}/3rdParty/Boost/lib \
        -lboost_filesystem-vc100-mt-1_49
        -lboost_system-vc100-mt-1_49
        -llibboost_filesystem-vc100-mt-1_49
        -llibboost_system-vc100-mt-1_49

        PRE_TARGETDEPS += \
        $${PCLDIR}/lib/pcl_common_release.lib \
        $${PCLDIR}/lib/pcl_io_release.lib \
        $${PCLDIR}/lib/pcl_filters_release.lib \
        $${PCLDIR}/lib/pcl_kdtree_release.lib \
        $${PCLDIR}/lib/pcl_registration_release.lib \
        $${PCLDIR}/lib/pcl_features_release.lib \
        $${PCLDIR}/lib/pcl_segmentation_release.lib \
        $${PCLDIR}/lib/pcl_surface_release.lib \
        $${PCLDIR}/lib/pcl_search_release.lib \
        $${PCLDIR}/3rdParty\FLANN\lib\flann_s.lib \
        $${PCLDIR}/3rdParty\Boost\lib\boost_filesystem-vc100-mt-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\boost_system-vc100-mt-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\libboost_filesystem-vc100-mt-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\libboost_system-vc100-mt-1_49.lib
        }else{

        LIBS += -L$${PCLDIR}/lib \
        -lpcl_common_debug \
        -lpcl_io_debug \
        -lpcl_filters_debug \
        -lpcl_kdtree_debug \
        -lpcl_registration_debug \
        -lpcl_features_debug \
        -lpcl_segmentation_debug \
        -lpcl_surface_debug \
        -lpcl_search_debug

        LIBS += -L$${PCLDIR}/3rdParty/FLANN/lib \
        -lflann_s-gd

        LIBS += -L$${PCLDIR}/3rdParty/Boost/lib \
        -lboost_filesystem-vc100-mt-gd-1_49
        -lboost_system-vc100-mt-gd-1_49
        -llibboost_filesystem-vc100-mt-gd-1_49
        -llibboost_system-vc100-mt-gd-1_49

        PRE_TARGETDEPS += \
        $${PCLDIR}/lib/pcl_common_debug.lib \
        $${PCLDIR}/lib/pcl_io_debug.lib \
        $${PCLDIR}/lib/pcl_filters_debug.lib \
        $${PCLDIR}/lib/pcl_kdtree_debug.lib \
        $${PCLDIR}/lib/pcl_registration_debug.lib \
        $${PCLDIR}/lib/pcl_features_debug.lib \
        $${PCLDIR}/lib/pcl_segmentation_debug.lib \
        $${PCLDIR}/lib/pcl_surface_debug.lib \
        $${PCLDIR}/lib/pcl_search_debug.lib \
        $${PCLDIR}/3rdParty\FLANN\lib\flann_s-gd.lib \
        $${PCLDIR}/3rdParty\Boost\lib\boost_filesystem-vc100-mt-gd-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\boost_system-vc100-mt-gd-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\libboost_filesystem-vc100-mt-gd-1_49.lib \
        $${PCLDIR}/3rdParty\Boost\lib\libboost_system-vc100-mt-gd-1_49.lib
        }
        message("PCL libraries found in $${PCLDIR}")
      } else {
        message("PCL libraries not found.")
      }



    DEFINES += WINDOWS
}
