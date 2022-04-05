cmake_minimum_required( VERSION 3.3 )
project( OpenGLDemos )

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )

# OpenGL
find_package( OpenGL REQUIRED )
set( ALL_LIBS ${OPENGL_LIBRARY} GLU glut )

add_executable( demo00 demo00/main.cpp )
target_link_libraries( demo00 ${ALL_LIBS} )
