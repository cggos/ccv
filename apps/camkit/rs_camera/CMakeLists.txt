cmake_minimum_required(VERSION 2.8)
project(rs_d435)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-w)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV COMPONENTS core highgui)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

find_package(realsense2)
if(realsense2_FOUND)
    include_directories(${realsense_INCLUDE_DIR})
endif()

add_executable(rs_d435 rs_d435i.cc)
target_link_libraries(rs_d435 ${OpenCV_LIBS} ${realsense2_LIBRARY})
