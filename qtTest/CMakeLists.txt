cmake_minimum_required(VERSION 2.8.9)
project(fabscan100)

find_package(Qt5Widgets REQUIRED)

add_executable(fabscan100 main.cpp)
install(TARGETS fabscan100 RUNTIME DESTINATION bin)

qt5_use_modules(fabscan100 Widgets)
