cmake_minimum_required(VERSION 2.8.3)
project(pcl_demo)

# set( PCL_DIR "/usr/local/share/pcl-1.7/" )
find_package(PCL REQUIRED COMPONENTS common io filters visualization)
if(PCL_FOUND)
    include_directories( ${PCL_INCLUDE_DIRS} )
    add_definitions( ${PCL_DEFINITIONS} )
endif()

add_executable(extract_domain_plane_ransac main.cpp extract_domain_plane_ransac.cpp)
target_link_libraries(extract_domain_plane_ransac ${PCL_LIBRARIES})