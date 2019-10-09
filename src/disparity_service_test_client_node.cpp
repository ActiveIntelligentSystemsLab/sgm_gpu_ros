#include "disparity_service_test_client.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity_service_test_client");
  sgm_gpu::DisparityServiceTestClient disparity_test_client;
  ros::spin();

  return 0;
}

