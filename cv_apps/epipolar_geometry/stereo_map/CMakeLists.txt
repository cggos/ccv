cmake_minimum_required(VERSION 2.8)
project(dense_stereo)

set(CMAKE_CXX_STANDARD 11)

include_directories( "/usr/include/eigen3" )

find_package( Pangolin )
if( Pangolin_FOUND )
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    link_libraries( ${Pangolin_LIBRARIES} )
endif()

find_package( OpenCV )
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    link_libraries( ${OpenCV_LIBS} )
endif()

# add_executable(dense_stereo dense_stereo.cpp)

add_executable(disparity disparity.cpp)
