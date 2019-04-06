#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

#include <memory>

namespace sgm_gpu_wrapper {

class SgmGpuNodelet : public nodelet::Nodelet
{
private:
  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  using StereoSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>;
  std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;

  ros::Publisher disparity_pub_;

  /**
   * @brief Parameter used in SGM algorithm
   */
  uint8_t sgm_p1_;

  /**
   * @brief Paramter used in SGM algorithm
   */
  uint8_t sgm_p2_;

  void stereoCallback(const sensor_msgs::ImageConstPtr &left_image_msg, const sensor_msgs::ImageConstPtr &right_image_msg, const sensor_msgs::CameraInfoConstPtr &left_info_msg, const sensor_msgs::CameraInfoConstPtr &right_info_msg);

public:
  virtual void onInit();
};

} // namespace sgm_gpu_wrapper
