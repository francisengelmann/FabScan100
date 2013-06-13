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
    "qextserialport-1.2beta2/src/*.h"
    "qextserialport-1.2beta2/src/*.cpp"
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Multimedia REQUIRED)
find_package(Qt5MultimediaWidgets REQUIRED)

include_directories(${Qt5Widgets_DEFINITIONS})
add_definitions(${Qt5Widgets_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#find_package(Eigen3 REQUIRED)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(fabscan ${fabscan_SRC})
install(TARGETS fabscan RUNTIME DESTINATION bin)
target_link_libraries(fabscan ${Qt5Core_LIBRARIES})

qt5_use_modules(fabscan Widgets)
qt5_use_modules(fabscan Multimedia)
qt5_use_modules(fabscan MultimediaWidgets)

qt5_wrap_ui(ui_mainwindow.h mainwindow.ui)
qt5_wrap_ui(ui_fscontrolpanel.h fscontrolpanel.ui)
qt5_wrap_ui(ui_fsdialog.h fsdialog.ui)
