include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_MODULE_PATH APPEND "${CMAKE_SOURCE_DIR}/cmake_modules/")
find_package(Eigen3)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc features2d video calib3d)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

## maths
add_executable(test_maths gtest_main.cpp test_math.cpp)
target_link_libraries(test_maths maths_cg gtest pthread ${OpenCV_LIBS})

## computer vision
add_executable(test_cv gtest_main.cpp test_cv.cpp)
target_link_libraries(test_cv cv_cg gtest pthread ${OpenCV_LIBS})

## kinematics
add_executable(test_kinematics gtest_main.cpp test_kinematics.cpp)
target_link_libraries(test_kinematics kinematics_cg gtest pthread ${OpenCV_LIBS})
