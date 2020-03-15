/***********************************************************************
  Copyright (C) 2020 Hironori Fujimoto

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
#ifndef SGM_GPU__SGM_GPU_NODE_H_
#define SGM_GPU__SGM_GPU_NODE_H_

#include "sgm_gpu/sgm_gpu.h"

#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace sgm_gpu
{

class SgmGpuNode
{
private:
  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<ros::NodeHandle> private_node_handle_;

  std::shared_ptr<image_transport::ImageTransport> image_transport_;

  std::shared_ptr<SgmGpu> sgm_;

  image_transport::SubscriberFilter left_image_sub_;
  image_transport::SubscriberFilter right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  using StereoSynchronizer = message_filters::TimeSynchronizer
  <
    sensor_msgs::Image, sensor_msgs::Image, 
    sensor_msgs::CameraInfo, sensor_msgs::CameraInfo
  >;
  std::shared_ptr<StereoSynchronizer> stereo_synchronizer_;

  ros::Publisher disparity_pub_;
  
  void stereoCallback(
    const sensor_msgs::ImageConstPtr &left_image_msg, 
    const sensor_msgs::ImageConstPtr &right_image_msg, 
    const sensor_msgs::CameraInfoConstPtr &left_info_msg, 
    const sensor_msgs::CameraInfoConstPtr &right_info_msg
  );

public:
  SgmGpuNode();
};

} // namespace sgm_gpu

#endif

