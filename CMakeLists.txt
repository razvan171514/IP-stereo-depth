cmake_minimum_required(VERSION 3.23)
project(Strereo_depth)

set(CMAKE_CXX_STANDARD 14)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( Strereo_depth main.cpp stereo.cpp stereo.h transform.cpp transform.h)
target_link_libraries( Strereo_depth ${OpenCV_LIBS} )
