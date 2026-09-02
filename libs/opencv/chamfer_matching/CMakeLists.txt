project(chamfer_matching)

add_executable(chamfer_matching chamfer_matching.cpp)
target_compile_features(chamfer_matching PRIVATE cxx_std_17)
target_link_libraries(chamfer_matching PRIVATE ${OpenCV_LIBS})
