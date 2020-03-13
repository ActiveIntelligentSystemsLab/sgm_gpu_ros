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

#ifndef DEBUG_H_
#define DEBUG_H_

#include <iostream>
#include <stdio.h>
#include "sgm_gpu/configuration.h"

namespace sgm_gpu
{

void debug_log(const char *str);

}

#endif /* DEBUG_H_ */
