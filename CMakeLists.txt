# CMake>=3.8 supports CUDA C++ as intrinsically supported language
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(sgm_gpu CUDA CXX)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
	disparity_srv
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
  include/sgm_gpu
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sgm_gpu_nodelet.cu
  src/sgm_gpu/costs.cu
  src/sgm_gpu/debug.cu
  src/sgm_gpu/disparity_method.cu
  src/sgm_gpu/hamming_cost.cu
  src/sgm_gpu/left_right_consistency.cu
  src/sgm_gpu/median_filter.cu
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sgm_gpu_node ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_server
  src/${PROJECT_NAME}_server_node.cu
  src/${PROJECT_NAME}_server.cu
  src/sgm_gpu/costs.cu
  src/sgm_gpu/debug.cu
  src/sgm_gpu/disparity_method.cu
  src/sgm_gpu/hamming_cost.cu
  src/sgm_gpu/left_right_consistency.cu
  src/sgm_gpu/median_filter.cu
)
add_dependencies(${PROJECT_NAME}_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(disparity_service_test_client 
  src/disparity_service_test_client_node.cpp
  src/disparity_service_test_client.cpp
)
add_dependencies(disparity_service_test_client ${catkin_EXPORTED_TARGETS})
target_link_libraries(disparity_service_test_client ${catkin_LIBRARIES})
