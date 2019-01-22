cmake_minimum_required(VERSION 2.4)
project(sgm_cuda)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( CUDA REQUIRED )
set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -lineinfo
    -gencode=arch=compute_30,code=sm_30
    -gencode=arch=compute_35,code=sm_35
    -gencode=arch=compute_50,code=sm_50
    -gencode=arch=compute_52,code=sm_52
    )


set(CUDA_SRC_LISTS median_filter.cu hamming_cost.cu disparity_method.cu debug.cu costs.cu)

# cuda_add_executable(sgm_cuda main.cu ${CUDA_SRC_LISTS})

cuda_add_library( sgm_cuda SHARED ${CUDA_SRC_LISTS} )

target_link_libraries( sgm_cuda ${OpenCV_LIBS} )
