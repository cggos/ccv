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

set(CIMG_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/CImg.h)
if(NOT EXISTS ${CIMG_HEADER})
    file(DOWNLOAD
            https://raw.githubusercontent.com/dtschump/CImg/master/CImg.h
            ${CIMG_HEADER}
            SHOW_PROGRESS)
endif()

add_executable(test_cvkit gtest_main.cpp test_image.cpp)
target_link_libraries(test_cvkit cvkit gtest pthread -lX11)

## maths
add_executable(test_maths gtest_main.cpp test_math.cpp)
target_link_libraries(test_maths maths_cg gtest pthread ${OpenCV_LIBS})

## computer vision
add_executable(test_cv gtest_main.cpp test_cv.cpp)
target_link_libraries(test_cv cv_cg gtest pthread ${OpenCV_LIBS})

## kinematics
add_executable(test_kinematics gtest_main.cpp test_kinematics.cpp)
target_link_libraries(test_kinematics kinematics_cg gtest pthread ${OpenCV_LIBS})
