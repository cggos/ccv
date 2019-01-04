cmake_minimum_required(VERSION 2.8)
project(demo01)

add_executable(demo01 demo01.cpp)
target_link_libraries(demo01 -lfftw3f -lm -lpthread -lX11)