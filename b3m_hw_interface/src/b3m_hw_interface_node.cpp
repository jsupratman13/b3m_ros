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

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <vector>
#include <future>

class ParallelCombinedRobotHW : public combined_robot_hw::CombinedRobotHW
{
public:
  bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) override
  {
    root_nh_ = nh;
    robot_hw_nh_ = nh_private;
    std::vector<std::string> robots;
    std::string param_name = "robot_hardware";
    if (!robot_hw_nh_.getParam(param_name, robots))
    {
      ROS_ERROR_STREAM("Could not find '" << param_name << "' parameter (namespace: " << nh_private.getNamespace()
                                          << ").");
      return false;
    }
    std::vector<std::future<bool>> futures;
    futures.reserve(robots.size());
    for (const auto& robot : robots)
    {
      futures.push_back(std::async(std::launch::async, &ParallelCombinedRobotHW::loadRobotHW, this, robot));
    }
    for (auto& future : futures)
    {
      if (!future.get())
      {
        return false;
      }
    }
    return true;
  }
  void read(const ros::Time& time, const ros::Duration& period) override
  {
    std::vector<std::future<void>> futures;
    futures.reserve(robot_hw_list_.size());
    for (auto& robot_hw : robot_hw_list_)
    {
      futures.push_back(std::async(std::launch::async, &hardware_interface::RobotHW::read, robot_hw, time, period));
    }
    for (auto& future : futures)
    {
      future.get();
    }
  }
  void write(const ros::Time& time, const ros::Duration& period) override
  {
    std::vector<std::future<void>> futures;
    futures.reserve(robot_hw_list_.size());
    for (auto& robot_hw : robot_hw_list_)
    {
      futures.push_back(std::async(std::launch::async, &hardware_interface::RobotHW::write, robot_hw, time, period));
    }
    for (auto& future : futures)
    {
      future.get();
    }
  }
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list) override
  {
    std::vector<std::future<bool>> futures;
    futures.reserve(robot_hw_list_.size());
    for (const auto& robot_hw : robot_hw_list_)
    {
      futures.push_back(std::async(std::launch::async, [this, &start_list, &stop_list, robot_hw]() {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw);
        filterControllerList(stop_list, filtered_stop_list, robot_hw);

        return robot_hw->prepareSwitch(filtered_start_list, filtered_stop_list);
      }));
    }

    for (auto& future : futures)
    {
      if (!future.get())
      {
        ROS_ERROR_STREAM("Failed to prepare switch for one of the robot hardware.");
        return false;
      }
    }

    return true;
  }
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override
  {
    std::vector<std::future<void>> futures;
    futures.reserve(robot_hw_list_.size());
    for (const auto& robot_hw : robot_hw_list_)
    {
      futures.push_back(std::async(std::launch::async, [this, &start_list, &stop_list, robot_hw]() {
        std::list<hardware_interface::ControllerInfo> filtered_start_list;
        std::list<hardware_interface::ControllerInfo> filtered_stop_list;

        filterControllerList(start_list, filtered_start_list, robot_hw);
        filterControllerList(stop_list, filtered_stop_list, robot_hw);

        robot_hw->doSwitch(filtered_start_list, filtered_stop_list);
      }));
    }

    for (auto& future : futures)
    {
      future.get();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "b3m_hw_interface_node");
  ros::NodeHandle nh, nh_private("~");

  ParallelCombinedRobotHW hw;
  if (!hw.init(nh, nh_private))
  {
    ROS_FATAL_STREAM("Initializing hardware interfaces failed");
    ros::shutdown();
    return 1;
  }
  ROS_INFO_STREAM("Initializing hardware interfaces succeeded, starting hardware interface");

  controller_manager::ControllerManager cm(&hw, nh);
  ros::Rate rate(100);  // 100Hz update rate

  while (ros::ok())
  {
    hw.read(ros::Time::now(), rate.expectedCycleTime());
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    hw.write(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  return 0;
}
