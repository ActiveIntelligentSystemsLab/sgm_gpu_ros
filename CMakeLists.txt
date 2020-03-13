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
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sgm_gpu_nodelet.cu
  src/costs.cu
  src/debug.cu
  src/disparity_method.cu
  src/hamming_cost.cu
  src/left_right_consistency.cu
  src/median_filter.cu
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sgm_gpu_node ${catkin_LIBRARIES})

