cmake_minimum_required(VERSION 2.8)
project(cgocv)

set(cgocv_VERSION_MAJOR 1)
set(cgocv_VERSION_MINOR 0)
configure_file(
    ${PROJECT_SOURCE_DIR}/cmake_modules/cgocv_config.h.in
    ${PROJECT_BINARY_DIR}/cgocv_config.h)

option(BUILD_DOCS      "Create and install the HTML based API documentation (requires Doxygen)" OFF)
option(BUILD_TEST      "Build Unit Test Program for the project (requires GTest)" OFF)
option(WITH_OPENCV_PCL "Build with OpenCV and PCL" ON)
option(WITH_ROS        "Build with ROS" OFF)

if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -O3")
else(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -O0 -g -Wall")
    add_definitions(-D__USER_DEBUG__)
endif(CMAKE_BUILD_TYPE MATCHES "Release")

if(WITH_ROS)
    find_package(catkin REQUIRED)
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cgocv)
    include_directories(${catkin_INCLUDE_DIRS})
endif()

include_directories(include ${PROJECT_BINARY_DIR})
set(SRC_LIST
    src/image.cc
    src/features2d.cc
    src/features2d_fast.cc
    src/mathtools.cc)

if(WITH_OPENCV_PCL)
    find_package(OpenCV REQUIRED)
    if(OpenCV_FOUND)
        include_directories(${OpenCV_INCLUDE_DIRS})
        link_libraries(${OpenCV_LIBS})
    endif()

    find_package(PCL REQUIRED COMPONENTS common io)
    if(PCL_FOUND)
        include_directories(${PCL_INCLUDE_DIRS})
        link_libraries(${PCL_LIBRARIES})
        add_definitions(${PCL_DEFINITIONS})
    endif()
    
    list(APPEND SRC_LIST src/stereo_camera.cc)
endif()

add_library(cgocv SHARED ${SRC_LIST})

if(BUILD_DOCS)
    include(cmake_modules/build_doxyfile.cmake)
endif()

if(BUILD_TEST)
    add_subdirectory(test)
endif()

if(WITH_ROS)
    install(TARGETS cgocv
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".cc" EXCLUDE)
else()
    install(TARGETS cgocv 
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
    install(FILES ${PROJECT_BINARY_DIR}/cgocv_config.h DESTINATION include)
    install(DIRECTORY include/cgocv
        DESTINATION include
        PATTERN ".cc" EXCLUDE)
endif()


