cmake_minimum_required(VERSION 3.10)
project(ccv)

set(CCV_VERSION_MAJOR 1)
set(CCV_VERSION_MINOR 0)

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/ccv_config.h.in
  ${PROJECT_BINARY_DIR}/ccv_config.h)

option(BUILD_DOCS "Create and install the HTML based API documentation (requires Doxygen)" OFF)
option(BUILD_TEST "Build Unit Test Program for the project (requires GTest)" OFF)
option(WITH_OPENCV "Build with OpenCV" ON)
option(WITH_PCL "Build with PCL" OFF)
option(WITH_GL "Build with OpenGL and Pangolin" ON)

set(CMAKE_CXX_STANDARD 17) # -std=gnu++11

add_definitions(-w)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3")
elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O0 -g -Wall")
  add_definitions(-D__USER_DEBUG__)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL Clang OR CMAKE_CXX_COMPILER_ID STREQUAL AppleClang)
  string(APPEND CMAKE_CXX_FLAGS " -Wno-c++11-narrowing")
endif()

include_directories(include ${PROJECT_BINARY_DIR})

# find_package(OpenMP)
# if(OpenMP_FOUND)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

# find_package(Boost REQUIRED COMPONENTS system)
# if(Boost_FOUND)
# include_directories(${Boost_INCLUDE_DIR})
# link_directories(${Boost_LIBRARY_DIR})
# endif()
set(CV_SRC_LIST
  src/maths/mathtools.cc
  src/cv/image.cc
  src/cv/features2d.cc
  src/cv/features2d_fast.cc
  src/cv/undistort.cpp
  src/cv/calib3d.cpp
  src/cv/corner_detector.cpp
  src/cv/visual_tracking.cpp
  src/cv/image_filtering.cpp
)

if(WITH_OPENCV)
  find_package(OpenCV REQUIRED)

  if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_libraries(${OpenCV_LIBS})
  endif()

  list(APPEND CV_SRC_LIST
    src/cv/image_ocv.cc
    src/cv/stereo_camera.cc
    src/cv/pointcloud3d.cc
  )
endif()

if(WITH_PCL)
  find_package(PCL REQUIRED COMPONENTS common io)

  if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_libraries(${PCL_LIBRARIES})
    add_definitions(${PCL_DEFINITIONS} -DWITH_PCL)
  endif()
endif()

if(WITH_GL)
  find_package(Pangolin)

  if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
    link_libraries(${Pangolin_LIBRARIES})
    add_definitions(-DWITH_GL)
  endif()
endif()

# # maths
add_library(maths_cg SHARED
  src/maths/math_basics.cpp
  src/maths/matrix.cpp
  src/maths/vector.cpp
  src/maths/svd_fulluv.cpp)

# # kinematics
add_library(kinematics_cg SHARED
  src/kinematics/rotation_matrix.cpp
  src/kinematics/quarternion.cpp
  src/kinematics/convertor.cpp)
target_link_libraries(kinematics_cg maths_cg)

# # computer vision
include_directories(/usr/local/include/eigen3) # will be removed !!!!!!!!!!!!

add_library(cv_cg SHARED ${CV_SRC_LIST})
target_link_libraries(cv_cg kinematics_cg)

# ############################## unit test ###############################
if(BUILD_TEST)
  enable_testing()
  find_package(GTest)

  if(GTest_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
    add_subdirectory(unit_test)
  endif()
endif()

# ############################## api docs ###############################
if(BUILD_DOCS)
  include(cmake/build_doxyfile.cmake)
endif()

# ############################## install ###############################
install(TARGETS cv_cg maths_cg kinematics_cg
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
install(FILES ${PROJECT_BINARY_DIR}/ccv_config.h DESTINATION include/ccv)
install(DIRECTORY
  include/ccv/common
  include/ccv/cv
  include/ccv/maths
  include/ccv/kinematics
  DESTINATION include
  PATTERN ".cc" EXCLUDE)
