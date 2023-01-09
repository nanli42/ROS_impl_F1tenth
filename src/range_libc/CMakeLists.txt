cmake_minimum_required(VERSION 3.3)
project(range_libc)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES range_libc
)

add_subdirectory(include/vendor/gflags)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(include)

add_definitions(-DUSE_CUDA=1)
add_definitions(-DCHUNK_SIZE=262144)
add_definitions(-DNUM_THREADS=256)

find_package(CUDA REQUIRED)
set( CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}" "-arch=sm_75 --ptxas-options=-v" )

cuda_add_library(range_libc src/kernels.cu
                            include/vendor/lodepng/lodepng.cpp
)

target_link_libraries(range_libc gflags pthread)

install(DIRECTORY include/range_libc/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
