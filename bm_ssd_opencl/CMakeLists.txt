cmake_minimum_required(VERSION 2.6)
project(stereo_depth_cl)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 -g")

find_package(OpenCL REQUIRED)
if(OpenCL_FOUND)
	include_directories(${OpenCL_INCLUDE_DIRS})
else(OpenCL_FOUND)
	message(">> OpenCL not found. OpenCL is required for GPGPU/FPGA/Accelerator use.")
endif(OpenCL_FOUND)

link_directories(${OpenCL_LIBRARIES})

message("=========" ${OpenCL_LIBRARIES})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(host_main host_main.cpp)
target_link_libraries(host_main ${OpenCL_LIBRARIES} png)

