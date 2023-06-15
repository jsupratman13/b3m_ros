/*********************************************************************
 * Copyright (c) 2023
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
 ********************************************************************/

#include "b3m_driver/b3m_interface.hpp"
#include "b3m_driver/b3m_map.hpp"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sample_b3m_driver_node");
  ros::NodeHandle nh;

  b3m_driver::B3MInterface b3m;
  try
  {
    b3m.connect("/dev/ttyUSB0", 1500000);
    ROS_INFO_STREAM("successfully connected to /dev/ttyUSB0");
  }
  catch (std::runtime_error& e)
  {
    ROS_FATAL_STREAM(e.what());
    ros::shutdown();
    return 1;
  }
  // reset b3m
  b3m.reset(1);
  ros::Duration(1).sleep();
  //  set b3m free mode
  b3m.setServoMode(1, OPTIONS_RUN_FREE);
  //  set b3m position control mode
  b3m.setServoMode(1, OPTIONS_CONTROL_POSITION);
  b3m.setTrajectoryType(1, TRAJECTORY_EVEN);
  //  set b3m normal mode
  b3m.setServoMode(1, OPTIONS_RUN_NORMAL);
  //  set b3m position
  b3m.setDesiredPosition(1, -3000);
  ros::Duration(1).sleep();
  ROS_INFO_STREAM("current position: " << b3m.getCurrentPosition(1));
}
