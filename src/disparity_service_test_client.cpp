#include "disparity_service_test_client.h"

#include <image_transport/camera_common.h>
#include <sgm_gpu/EstimateDisparity.h>

namespace sgm_gpu
{
  DisparityServiceTestClient::DisparityServiceTestClient()
  {
    ros::NodeHandle node_handle;

    disparity_service_client_ = node_handle.serviceClient<EstimateDisparity>("estimate_disparity");
    disparity_service_client_.waitForExistence();

    ros::NodeHandle private_node_handle("~");
    disparity_publisher_ = private_node_handle.advertise<stereo_msgs::DisparityImage>("disparity", 10);

    image_transport_.reset(new image_transport::ImageTransport(node_handle));

    // Find base_topic
    std::string left_base_topic = node_handle.resolveName("left_image");
    std::string right_base_topic = node_handle.resolveName("right_image");

    left_image_subscriber_.subscribe(*image_transport_, left_base_topic, 10);
    right_image_subscriber_.subscribe(*image_transport_, right_base_topic, 10);

    // Find CameraInfo topic from corresponded Image topic and subscribe it
    std::string left_caminfo_topic = image_transport::getCameraInfoTopic(left_base_topic);
    std::string right_caminfo_topic = image_transport::getCameraInfoTopic(right_base_topic);
    left_caminfo_subscriber_.subscribe(node_handle, left_caminfo_topic, 10);
    right_caminfo_subscriber_.subscribe(node_handle, right_caminfo_topic, 10);

    stereo_synchronizer_.reset(new StereoSynchronizer(left_image_subscriber_, right_image_subscriber_, left_caminfo_subscriber_, right_caminfo_subscriber_, 10));
    stereo_synchronizer_->registerCallback(&DisparityServiceTestClient::stereoCallback, this);
  }

  void DisparityServiceTestClient::stereoCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info)
  {
    EstimateDisparity service_object;

    service_object.request.left_image = *left_image;
    service_object.request.right_image = *right_image;
    service_object.request.left_camera_info = *left_camera_info;
    service_object.request.right_camera_info = *right_camera_info;

    bool success = disparity_service_client_.call(service_object);

    if (success) 
    {
      ROS_INFO_STREAM("Service call was success. Publish result...");
      disparity_publisher_.publish(service_object.response.disparity);
    }
    else
    {
      ROS_INFO_STREAM("Service call was failed!");
    }
  }
}
