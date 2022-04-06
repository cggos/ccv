cmake_minimum_required(VERSION 2.8.3)
project(stereo_calib_rectify)

set(CMAKE_BUILD_TYPE Release)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stereo_calib_rectify_nodelet
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_geometry image_transport message_filters nodelet roscpp rospy sensor_msgs std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stereo_calib_rectify_nodelet src/capture_cam_stereo_nodelet.cc src/calib_fisheye_stereo.cc)
target_link_libraries(stereo_calib_rectify_nodelet ${catkin_LIBRARIES})

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(stereo_calib_rectify_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
