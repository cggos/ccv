cmake_minimum_required(VERSION 2.8)
project( cam_v4l2 )

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED COMPONENTS highgui)
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    link_libraries( ${OpenCV_LIBS} )
endif()

add_definitions(-D__USER_DEBUG__)

include_directories(./)

add_library(cam_v4l2 SHARED cam_v4l2.cpp)

add_executable(cam_v4l2_demo main.cpp)
target_link_libraries(cam_v4l2_demo cam_v4l2)