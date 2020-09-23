cmake_minimum_required(VERSION 2.8)
project(pangolin_demo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-w)

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
  include_directories(${Pangolin_INCLUDE_DIRS})
  link_libraries(${Pangolin_LIBRARIES})
endif()

find_package(OpenCV REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

add_executable(demo01 demo01.cc)
add_executable(demo02 demo02.cc)
add_executable(demo03 demo03.cc)
add_executable(demo04 demo04.cc)
target_link_libraries(demo04 ${OpenCV_LIBS})
add_executable(demo05 demo05.cc)
add_executable(demo06 demo06.cc)
