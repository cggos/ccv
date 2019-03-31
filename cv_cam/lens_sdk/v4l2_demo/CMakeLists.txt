cmake_minimum_required(VERSION 2.8)
project( v4l2_demo )

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV REQUIRED COMPONENTS highgui)
if( OpenCV_FOUND )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    link_libraries( ${OpenCV_LIBS} )
endif()

add_executable(v4l2_demo main.cpp)