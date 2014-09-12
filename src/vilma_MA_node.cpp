

/**
 *
	 @brief This is main program of the package VILMA_MA
 @author olmer Garcia olmerg@gmail.com

 * Copyright 2013 Olmer Garcia Bedoya
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <math.h>
#include <stdlib.h>
#include "ros/ros.h"

#include "protocolo.h"
#include "vilma_MA_ROS.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/asio/deadline_timer.hpp>




/**
*@brief This is main program of the package VILMA_MA
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  vilma_MA_ROS vilma_node;

	
  ros::spin();

  return 0;
}
