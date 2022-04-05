# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
  include_directories(${Pangolin_INCLUDE_DIRS})
endif()

find_package(OpenCV REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

add_executable(slam_visualization main.cpp slam_visualizer.cpp)
target_link_libraries(slam_visualization ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
