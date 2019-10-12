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

#include <ros/ros.h>
#include <disparity_srv/EstimateDisparity.h>

namespace sgm_gpu {

class SgmGpuServer
{
private:
  /**
   * @brief ROS service server provide EstimateDisparity service
   */
  ros::ServiceServer disparity_server_;

  /**
   * @brief Parameter used in SGM algorithm
   * See SGM paper.
   */
  uint8_t sgm_p1_;

  /**
   * @brief Paramter used in SGM algorithm
   * See SGM paper.
   */
  uint8_t sgm_p2_;
  
  /**
   * @brief Check left-right consistency if true
   */
  bool check_consistency_;
  
  /**
   * @brief Callback of EstimateDisparity service
   */
  bool disparityServiceCallback(disparity_srv::EstimateDisparity::Request& request, disparity_srv::EstimateDisparity::Response& response);

public:
  SgmGpuServer();
};

} // namespace sgm_gpu

