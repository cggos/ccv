cmake_minimum_required(VERSION 2.8)
project(compute_orb)

set(CMAKE_CXX_STANDARD 11)

find_package( OpenCV )
if( OpenCV_FOUND )
	include_directories( ${OpenCV_INCLUDE_DIRS} )
	link_libraries( ${OpenCV_LIBS} )
endif()

add_executable( compute_orb_01 compute_orb_01.cpp )
add_executable( compute_orb_02 compute_orb_02.cpp )
