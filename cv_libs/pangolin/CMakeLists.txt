cmake_minimum_required(VERSION 2.8)
project(pangolin_demo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-w)

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
   include_directories(${Pangolin_INCLUDE_DIRS})
endif()
list(APPEND LIBS ${Pangolin_LIBRARIES})

link_libraries(${LIBS})

add_executable(demo01 demo01.cc)
add_executable(demo02 demo02.cc)
add_executable(demo03 demo03.cc)
