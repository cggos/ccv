cmake_minimum_required(VERSION 3.20)
project(ccv)

set(CCV_VERSION_MAJOR 1)
set(CCV_VERSION_MINOR 0)
set(CCV_VERSION_PATCH 0)

option(BUILD_DOCS "Create and install the HTML based API documentation (requires Doxygen)" OFF)
option(BUILD_TEST "Build Unit Test Program for the project (requires GTest)" OFF)

option(WITH_OPENCV "Build with OpenCV" ON)
option(WITH_PCL "Build with PCL" OFF)
option(WITH_GL "Build with OpenGL and Pangolin" ON)

add_subdirectory(hpc)

include_directories(hpc/include)

set(Eigen3_DIR "$ENV{CG_THIRDPARTY}/${PLATFORM_ARCH}/eigen3/share/eigen3/cmake/")
find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

message("================== " ${EIGEN3_INCLUDE_DIR})

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

set(CMAKE_INSTALL_PREFIX "/opt/ccv")

include_directories(include)

configure_file(
  ${PROJECT_SOURCE_DIR}/cmake/ccv_config.h.in
  ${PROJECT_BINARY_DIR}/ccv_config.h)

# maths
add_library(${PROJECT_NAME}_maths SHARED
  src/maths/math_basics.cpp
  src/maths/mathtools.cc
  src/maths/matrix.cpp
  src/maths/vector.cpp
  src/maths/svd_fulluv.cpp)

set(CV_SRC_LIST
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
  find_package(OpenCV REQUIRED core highgui imgproc features2d video calib3d)

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

# kinematics
add_library(${PROJECT_NAME}_kd SHARED
  src/kd/rotation_matrix.cpp
  src/kd/quarternion.cpp
  src/kd/convertor.cpp)
target_link_libraries(${PROJECT_NAME}_kd ${PROJECT_NAME}_maths)

add_library(${PROJECT_NAME} SHARED ${CV_SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_kd)

# ############################## unit test ###############################
if(BUILD_TEST)
  enable_testing()
  find_package(GTest)

  if(GTest_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS} /usr/local/include)
    link_directories(/usr/local/lib)
    add_subdirectory(unit_test)
  endif()
endif()

# ############################## api docs ###############################
if(BUILD_DOCS)
  include(cmake/build_doxyfile.cmake)
endif()

# ############################## install ###############################
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_maths ${PROJECT_NAME}_kd
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY
  include/ccv
  DESTINATION include
  PATTERN ".cc" EXCLUDE)

install(FILES ${PROJECT_BINARY_DIR}/ccv_config.h DESTINATION include/ccv)
