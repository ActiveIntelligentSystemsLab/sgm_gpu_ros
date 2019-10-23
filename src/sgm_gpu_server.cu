/***********************************************************************
  Copyright (C) 2019 Hironori Fujimoto

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

#include "sgm_gpu_server.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/opencv.hpp>

#include "disparity_method.h"

namespace sgm_gpu {

SgmGpuServer::SgmGpuServer()
{
  ros::NodeHandle private_node_handle("~");

  // Get parameters used in SGM algorithm
  // Default value from https://github.com/dhernandez0/sgm/blob/master/README.md
  sgm_p1_ = static_cast<uint8_t>(private_node_handle.param("p1", 6));
  sgm_p2_ = static_cast<uint8_t>(private_node_handle.param("p2", 96));

  check_consistency_ = private_node_handle.param("check_consistency", true);

  init_disparity_method(sgm_p1_, sgm_p2_);

  disparity_server_ = private_node_handle.advertiseService("estimate_disparity", &SgmGpuServer::disparityServiceCallback, this);
}

bool SgmGpuServer::disparityServiceCallback(disparity_srv::EstimateDisparity::Request& request, disparity_srv::EstimateDisparity::Response& response)
{
  // Even if image has 3 channels(RGB), cv_bridge convert it to greyscale
  cv_bridge::CvImagePtr left_image = cv_bridge::toCvCopy(request.left_image, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImagePtr right_image = cv_bridge::toCvCopy(request.right_image, sensor_msgs::image_encodings::MONO8);

  if (left_image->image.rows != right_image->image.rows || left_image->image.cols != right_image->image.cols)
  {
    ROS_ERROR_STREAM("Image dimension of left and right are not same\n" << 
                     "Left: " << left_image->image.rows << "x" << left_image->image.cols << "\n" <<
                     "Right: " << right_image->image.rows << "x" << right_image->image.cols);
    return false;
  }

  // sgm_gpu specify image size to divisible by 4
  bool need_resize = false;
  cv::Size original_size, resized_size; 
  original_size = cv::Size(left_image->image.cols, left_image->image.rows);
  resized_size = original_size;
  if (original_size.width % 4 != 0)
  {
    need_resize = true;
    resized_size.width = (original_size.width / 4 + 1) * 4;
  }
  if (original_size.height % 4 != 0)
  {
    need_resize = true;
    resized_size.height = (original_size.height / 4 + 1) * 4;
  }
  if (need_resize)
  {
    ROS_INFO_STREAM(
      "Resize image width and height to divisible by 4 before stereo matching\n" <<
      "The size with restored to original at publishing\n" <<
      "Original size: " << original_size.width << "x" << original_size.height << "\n" <<
      "Resized to: " << resized_size.width << "x" << resized_size.height
    );
    cv::resize(left_image->image, left_image->image, resized_size, 0, 0, cv::INTER_LINEAR);
    cv::resize(right_image->image, right_image->image, resized_size, 0, 0, cv::INTER_LINEAR);
  }

  float elapsed_time_ms;
  cv::Mat disparity_8u;
  compute_disparity_method(left_image->image, right_image->image, &disparity_8u, &elapsed_time_ms, check_consistency_);

  ROS_INFO("Elapsed time: %f [ms]", elapsed_time_ms);

  if (need_resize)
    cv::resize(disparity_8u, disparity_8u, original_size, 0, 0, cv::INTER_AREA);

  cv::Mat disparity_32f;
  disparity_8u.convertTo(disparity_32f, CV_32F);

  response.disparity.header = request.left_image.header;

  cv_bridge::CvImage disparity_converter(request.left_image.header, sensor_msgs::image_encodings::TYPE_32FC1, disparity_32f);
  disparity_converter.toImageMsg(response.disparity.image);

  image_geometry::StereoCameraModel stereo_model;
  stereo_model.fromCameraInfo(request.left_camera_info, request.right_camera_info);
  response.disparity.f = stereo_model.left().fx();
  response.disparity.T = stereo_model.baseline();

  response.disparity.min_disparity = 0.0;
  response.disparity.max_disparity = 128.0;
  
  response.disparity.delta_d = 1.0;

  return true;
}

} // namespace sgm_gpu

