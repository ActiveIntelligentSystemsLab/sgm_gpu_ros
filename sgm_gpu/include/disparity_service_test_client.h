#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <memory>

namespace sgm_gpu
{
  /**
   * \brief Test client of EstimateDisparity service
   */
  class DisparityServiceTestClient
  {
  private:
    std::shared_ptr<image_transport::ImageTransport> image_transport_;

    // Subscribers for stereo image and caminfo
    image_transport::SubscriberFilter left_image_subscriber_; 
    image_transport::SubscriberFilter right_image_subscriber_; 
    message_filters::Subscriber<sensor_msgs::CameraInfo> left_caminfo_subscriber_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> right_caminfo_subscriber_;

    using StereoSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>;
    /**
     * \brief Synchronizer for stereo image and camera info
     */
    std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;

    /**
     * \brief Publish result disparity
     */
    ros::Publisher disparity_publisher_;

    /**
     * \brief Service client for EstimateDisparity service
     */
    ros::ServiceClient disparity_service_client_;

    void stereoCallback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image, const sensor_msgs::CameraInfoConstPtr& left_camera_info, const sensor_msgs::CameraInfoConstPtr& right_camera_info);
  public:
    DisparityServiceTestClient();
  };
}
