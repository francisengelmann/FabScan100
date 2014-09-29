cmake_minimum_required(VERSION 2.8.9)
project(FabScan)

set(FabScan_VERSION_MAJOR 0)
set(FabScan_VERSION_MINOR 1)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
# MOC (Meta Object Compiler)
set(CMAKE_AUTOMOC ON)

#include all the source files
file(GLOB fabscan_SRC
    "*.h"
    "*.cpp"
)

set (qextserialport_FILES
    qextserialport-1.2rc/src/qextserialenumerator.h
    qextserialport-1.2rc/src/qextserialport.h
    qextserialport-1.2rc/src/qextserialenumerator_p.h
    qextserialport-1.2rc/src/qextserialport_global.h
    qextserialport-1.2rc/src/qextserialport_p.h
    qextserialport-1.2rc/src/qextserialport.cpp
    qextserialport-1.2rc/src/qextserialenumerator_linux.cpp
    qextserialport-1.2rc/src/qextserialenumerator.cpp
#    qextserialport-1.2rc/src/qextserialenumerator_osx.cpp
#    qextserialport-1.2rc/src/qextserialenumerator_win.cpp
    qextserialport-1.2rc/src/qextserialport_unix.cpp
#    qextserialport-1.2rc/src/qextserialport_win.cpp
#    qextserialport-1.2rc/src/qextserialenumerator_unix.cpp
)

find_library(UDEV_LIBS udev)
mark_as_advanced(UDEV_LIBS)

find_package(Qt5Core REQUIRED)
find_package(Qt5Multimedia REQUIRED)
find_package(Qt5MultimediaWidgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

include_directories(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#find_package(Eigen3 REQUIRED)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

qt5_wrap_ui(UIS_HDRS mainwindow.ui fscontrolpanel.ui fsdialog.ui)
qt5_add_resources(APP_RESOURCES shaders.qrc textures.qrc)

add_executable(fabscan ${fabscan_SRC} ${UIS_HDRS} ${qextserialport_FILES} ${APP_RESOURCES})
qt5_use_modules(fabscan Core Gui Multimedia MultimediaWidgets Widgets OpenGL)
install(TARGETS fabscan RUNTIME DESTINATION bin)
target_link_libraries(fabscan ${Qt5Core_LIBRARIES} GL ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${UDEV_LIBS})
