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

#include "right_disparity.h"
#include "configuration.h"

__global__ void ChooseRightDisparity(uint8_t *right_disparity, const uint16_t *smoothed_cost, const uint32_t rows, const uint32_t cols) {
  const int x = blockIdx.x*blockDim.x+threadIdx.x;
  const int y = blockIdx.y*blockDim.y+threadIdx.y;
  
  int min_cost_disparity = 0;
  uint16_t min_cost = smoothed_cost[y*cols*MAX_DISPARITY + x*MAX_DISPARITY + min_cost_disparity];
  
  // Prevent overflow of index
  /*
  int limit_disparity = MAX_DISPARITY;
  if (p_x + MAX_DISPARITY >= rows) {
    limit_disparity = p_x + MAX_DISPARITY - rows - 1;
  }
  */
  
  for (int d = 1; x + d < rows; d++) {
    uint16_t tmp_cost = smoothed_cost[y*cols*MAX_DISPARITY + (x+d)*MAX_DISPARITY + d];
    if (tmp_cost < min_cost) {
      min_cost = tmp_cost;
      min_cost_disparity = d;
    }
  }
  
  right_disparity[y*cols+x] = min_cost_disparity;
}
