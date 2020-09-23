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

add_executable(demo01 demo01.cc)
target_link_libraries(demo01 ${LIBS})