# CMake>=3.8 supports CUDA C++ as intrinsically supported language
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(sgm_gpu CUDA CXX)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  nodelet
  roscpp
  sensor_msgs
  stereo_msgs
)
find_package(OpenCV REQUIRED)

set(CUDA_NVCC_FLAGS
  ${CUDA_NVCC_FLAGS};
  -O3 -lineinfo
  -gencode=arch=compute_30,code=sm_30
  -gencode=arch=compute_35,code=sm_35
  -gencode=arch=compute_50,code=sm_50
  -gencode=arch=compute_52,code=sm_52
  -gencode=arch=compute_61,code=sm_61
)

catkin_package()

include_directories(
  include
  include/sgm
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sgm_gpu_nodelet.cu
  src/sgm/costs.cu
  src/sgm/debug.cu
  src/sgm/disparity_method.cu
  src/sgm/hamming_cost.cu
  src/sgm/median_filter.cu
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
