cmake_minimum_required( VERSION 2.8 )
project( mapping_mono_dense )

set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -fopenmp" )

include_directories( "/usr/include/eigen3" )

find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(mapping_mono_dense mapping_mono_dense.cpp )
target_link_libraries( mapping_mono_dense ${OpenCV_LIBS}${Sophus_LIBRARIES} )
