cmake_minimum_required(VERSION 2.8)
project(image_convertor)

find_package(OpenCV)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
endif()

add_executable(create_board_charuco      src/create_board_charuco.cpp)
add_executable(detect_board_charuco      src/detect_board_charuco.cpp)
add_executable(calib_board_charuco       src/calib_board_charuco.cpp)
