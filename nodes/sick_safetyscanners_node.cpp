// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*  Copyright (C) 2020, Tractonomy robotics BV, Kortrijk, Belgium
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file sick_micro_scan_ros_driver_node.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------

#include "sick_safetyscanners/SickSafetyscannersRos.h"
#include <rclcpp/rclcpp.hpp>

/*!
 * @brief main The Main node to start the ROS driver, this method is executed via launch file.
 * @param argc Number of arguments given.
 * @param argv Arguments which are given on startup of the main function.
 * @return If it runs successful, will always return 0 for the ROS driver.
 */

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  
  auto microscan3_ros = std::make_shared<sick::SickSafetyscannersRos>();
  
  rclcpp::spin(microscan3_ros);
  rclcpp::shutdown();
  return 0;
}
