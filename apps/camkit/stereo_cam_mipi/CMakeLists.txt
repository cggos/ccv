cmake_minimum_required(VERSION 2.8.3)
project(mipi_ros_wrapper)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -rdynamic -ldl")

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    rosconsole
    sensor_msgs
    tf
    tf2_ros
    image_geometry
    cv_bridge
)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(mini_ros_wrapper_node src/mipi_ros_node.cpp src/mipi_camera.cpp)
target_link_libraries(mini_ros_wrapper_node -ldl ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
  mini_ros_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
