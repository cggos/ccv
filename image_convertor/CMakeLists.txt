cmake_minimum_required(VERSION 2.8)
project(image_convertor)

find_package(OpenCV)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

add_executable(yuv2rgb yuv2rgb.cpp)
