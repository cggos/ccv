cmake_minimum_required(VERSION 2.8)
project(line_features)

find_package(OpenCV)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

add_executable(line_detect line_detect.cpp)
add_executable(line_match  line_match.cpp)
