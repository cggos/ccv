cmake_minimum_required(VERSION 2.8)
project(e2rt)

set(CMAKE_CXX_STANDARD 11)

include_directories( "/usr/include/eigen3" )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(e2rt e2rt.cpp)
target_link_libraries( e2rt ${Sophus_LIBRARIES} )
