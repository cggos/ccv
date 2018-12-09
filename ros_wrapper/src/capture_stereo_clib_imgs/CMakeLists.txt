cmake_minimum_required(VERSION 2.8.3)
project(capture_image)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  message_filters
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf_conversions
)

catkin_package(
  LIBRARIES capture_image
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_geometry image_transport message_filters nodelet pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs std_srvs tf tf2_ros tf_conversions
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/capture_image.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

