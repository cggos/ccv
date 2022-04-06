# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

# Sets the minimum version of CMake required to build the native library.

cmake_minimum_required(VERSION 3.4.1)

SET(CMAKE_BUILD_TYPE Release)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
#
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")
#add_definitions(-DCOMPILEDWITHC11)

add_definitions(-w)

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI})

find_library(log-lib log)

include_directories(${PROJECT_SOURCE_DIR}/JNITest ${PROJECT_SOURCE_DIR}/JNITest ${CPP_INCLUDE})

add_library(JNITest SHARED JNITest/com_ndk_test_JNITest.c)
target_link_libraries(JNITest ${log-lib})

#ENABLE_LANGUAGE(ASM)
#add_library(NEONTest SHARED NEONTest/com_ndk_test_NEONTest.cc) # NEONTest/Neon.S
#target_link_libraries(NEONTest ${log-lib})

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#message(STATUS "Configured OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "Configured OpenCV libraries: ${OpenCV_LIBS}")
#
#add_library(OpenCVTest SHARED OpenCVTest/com_ndk_test_OpenCVTest.cpp)
#target_link_libraries(OpenCVTest ${log-lib} ${OpenCV_LIBS})

##支持-std=gnu++11
#set(CMAKE_VERBOSE_MAKEFILE on)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
#
##动态方式加载
#add_library( lib_opencv SHARED IMPORTED )
#
##引入libopencv_java3.so文件
#set_target_properties(lib_opencv PROPERTIES IMPORTED_LOCATION
#        ${pathToOpenCv}/sdk/native/libs/${ANDROID_ABI}/libopencv_java3.so)