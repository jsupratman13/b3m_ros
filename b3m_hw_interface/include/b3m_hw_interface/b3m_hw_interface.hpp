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
#ifndef B3M_HW_INTERFACE_HPP_
#define B3M_HW_INTERFACE_HPP_

#include <controller_manager/controller_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <vector>

#include "b3m_driver/b3m_interface.hpp"

namespace b3m_hw_interface
{
using b3m_driver::B3MInterface;

class B3MHwInterface : public hardware_interface::RobotHW
{
public:
  bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;
  void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) override;

private:
  B3MInterface interface_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  std::size_t num_joints_;
  std::vector<std::string> joint_name_;
  std::vector<uint8_t> servo_id_;
  std::vector<double> gear_ratio_;
  std::vector<double> direction_;

  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;

  std::vector<double> command_;
  std::vector<double> prev_command_;
};  // class B3MHwInterface
}  // namespace b3m_hw_interface
#endif  // B3M_HW_INTERFACE_HPP_
