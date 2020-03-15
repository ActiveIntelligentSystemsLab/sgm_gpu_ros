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
#ifndef SGM_GPU__SGM_GPU_H_
#define SGM_GPU__SGM_GPU_H_

#include "sgm_gpu/configuration.h"

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/opencv.hpp>

namespace sgm_gpu
{

class SgmGpu
{
private:
  std::shared_ptr<ros::NodeHandle> private_node_handle_;
  /**
   * @brief Parameter used in SGM algorithm
   *
   * See SGM paper.
   */
  uint8_t p1_, p2_;
  /**
   * @brief Enable/disable left-right consistency check
   */
  bool check_consistency_;

  // Memory for disparity computation
  // d_: for device
  cudaStream_t stream1_, stream2_, stream3_;//, stream4, stream5, stream6, stream7, stream8;
  uint8_t *d_im0_;
  uint8_t *d_im1_;
  cost_t *d_transform0_;
  cost_t *d_transform1_;
  uint8_t *d_cost_;
  uint8_t *d_disparity_;
  uint8_t *d_disparity_filtered_uchar_;
  uint8_t *d_disparity_right_;
  uint8_t *d_disparity_right_filtered_uchar_;
  uint8_t *d_L0_;
  uint8_t *d_L1_;
  uint8_t *d_L2_;
  uint8_t *d_L3_;
  uint8_t *d_L4_;
  uint8_t *d_L5_;
  uint8_t *d_L6_;
  uint8_t *d_L7_;
  uint16_t *d_s_;

  bool memory_allocated_;

  uint32_t cols_, rows_;

  dim3 BLOCK_SIZE_;
  dim3 grid_size_;

  void allocateMemory(uint32_t cols, uint32_t rows);
  void freeMemory();

  /**
   * @brief Resize images to be width and height divisible by 4 for limit of CUDA code
   */
  void resizeToDivisibleBy4(cv::Mat& left_image, cv::Mat& right_image);

  void convertToMsg(
    const cv::Mat_<unsigned char>& disparity, 
    const sensor_msgs::CameraInfo& left_camera_info,
    const sensor_msgs::CameraInfo& right_camera_info,
    stereo_msgs::DisparityImage& disparity_msg
  );

public:
  /**
   * @brief Constructor which use namespace <parent>/libsgm_gpu for ROS param
   */
  SgmGpu(const ros::NodeHandle& parent_node_handle);
  ~SgmGpu();

  bool computeDisparity(
    const sensor_msgs::Image& left_image, 
    const sensor_msgs::Image& right_image,
    const sensor_msgs::CameraInfo& left_camera_info,
    const sensor_msgs::CameraInfo& right_camera_info,
    stereo_msgs::DisparityImage& disparity_msg
  );
};

} // namespace sgm_gpu

#endif // SGM_GPU__SGM_GPU_H_

