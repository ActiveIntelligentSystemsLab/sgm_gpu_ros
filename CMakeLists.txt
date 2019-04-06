# CMake>=3.8 supports CUDA C++ as intrinsically supported language
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(sgm_gpu_ros_wrapper CUDA CXX)

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
)

catkin_package()

include_directories(
  include
  sgm
  ${catkin_INCLUDE_DIRS}
)

set(NODELET_NAME sgm_gpu_nodelet)
add_library(${NODELET_NAME}
  src/${NODELET_NAME}.cu
  sgm/costs.cu
  sgm/debug.cu
  sgm/disparity_method.cu
  sgm/hamming_cost.cu
  sgm/median_filter.cu
)
target_link_libraries(${NODELET_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)