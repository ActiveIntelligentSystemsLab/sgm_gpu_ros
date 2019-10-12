cmake_minimum_required(VERSION 2.8.3)
project(disparity_srv)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  stereo_msgs
)

add_service_files(FILES EstimateDisparity.srv)
generate_messages(DEPENDENCIES
  sensor_msgs
  stereo_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime 
  sensor_msgs
  stereo_msgs
)

