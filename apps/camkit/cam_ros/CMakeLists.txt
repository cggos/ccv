cmake_minimum_required(VERSION 2.8.3)
project(camera_roswrapper)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf)

find_package(OpenCV REQUIRED core imgproc highgui videoio)

generate_dynamic_reconfigure_options(
    cfg/Camera.cfg
    cfg/CameraStereo.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf)

add_subdirectory(cam_v4l2)

include_directories(cam_v4l2 ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(camera src/camera.cpp)
add_dependencies(camera ${PROJECT_NAME}_gencfg)
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(camera_timer src/camera_timer.cpp)
target_link_libraries(camera_timer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(camera_v4l2 src/camera_v4l2.cpp)
target_link_libraries(camera_v4l2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} cam_v4l2)

add_executable(camera_polling src/camera_polling.cpp)
target_link_libraries(camera_polling ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(camera_stereo src/camera_stereo.cpp)
add_dependencies(camera_stereo ${PROJECT_NAME}_gencfg)
target_link_libraries(camera_stereo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS camera camera_timer camera_polling camera_stereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir bag config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
