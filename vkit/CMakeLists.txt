cmake_minimum_required(VERSION 3.1)
project(vikit_cg)

option(BUILD_TEST "Build Unit Test Program for the project (requires GTest)" OFF)
option(BUILD_DOC  "Create and install the HTML based API documentation (requires Doxygen)" OFF)

set(CMAKE_CXX_STANDARD 11) # -std=gnu++11

add_definitions(-w)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3") # -std=c++11
else(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O0 -g -Wall")
    add_definitions(-D__USER_DEBUG__)
endif(CMAKE_BUILD_TYPE MATCHES "Release")

#find_package(OpenMP)
#if(OpenMP_FOUND)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

#find_package(Boost REQUIRED COMPONENTS system)
#if(Boost_FOUND)
#    include_directories(${Boost_INCLUDE_DIR})
#    link_directories(${Boost_LIBRARY_DIR})
#endif()

include_directories(include)

add_subdirectory(3rdparty/fast)
include_directories(3rdparty/fast/include)

## maths
add_library(maths_cg SHARED
        src/maths/math_basics.cpp
        src/maths/matrix.cpp
        src/maths/vector.cpp
        src/maths/svd_fulluv.cpp)

## kinematics
add_library(kinematics_cg SHARED
        src/kinematics/rotation_matrix.cpp
        src/kinematics/quarternion.cpp
        src/kinematics/convertor.cpp)
target_link_libraries(kinematics_cg maths_cg)

## computer vision
include_directories(/usr/include/eigen3) # will be removed !!!!!!!!!!!!
add_library(cv_cg SHARED
        src/cv/types.cpp
        src/cv/undistort.cpp
        src/cv/calib3d.cpp
        src/cv/corner_detector.cpp
        src/cv/visual_tracking.cpp
        src/cv/image_filtering.cpp)
target_link_libraries(cv_cg kinematics_cg fast)

############################### unit test ###############################
if(BUILD_TEST)
    enable_testing()
    find_package(GTest)
    if(GTest_FOUND)
        include_directories(${GTEST_INCLUDE_DIRS})
        add_subdirectory(unit_test)
    endif()
endif()

############################### api docs ###############################
if(BUILD_DOC)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "NOT Found doxygen which is needed to build the documentation.")
    else()
        message(STATUS "Found doxygen, run `make doc` to build documentation")
        set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/config/Doxyfile.config)
        set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        configure_file(${doxyfile_in} ${doxyfile} @ONLY)
        add_custom_target(doc
                COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
                WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
    endif()
endif()

############################### install ###############################
INSTALL(TARGETS maths_cg kinematics_cg
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY include/maths include/cv include/kinematics
        DESTINATION include)
install(FILES include/common.h DESTINATION include)