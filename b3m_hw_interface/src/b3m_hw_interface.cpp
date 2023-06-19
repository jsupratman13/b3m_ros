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

#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <pluginlib/class_list_macros.hpp>
#include "b3m_driver/b3m_map.hpp"
#include "b3m_hw_interface/b3m_hw_interface.hpp"

namespace b3m_hw_interface
{

bool B3MHwInterface::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh)
{
  std::string port;
  int baudrate;
  robot_hw_nh.getParam("port", port);
  robot_hw_nh.getParam("baudrate", baudrate);
  try
  {
    interface_.connect(port, baudrate);
    ROS_INFO_STREAM("successfully connected to " << port);
  }
  catch (std::runtime_error& e)
  {
    ROS_FATAL_STREAM(e.what());
    ros::shutdown();
    return false;
  }

  XmlRpc::XmlRpcValue joints_param;
  if (robot_hw_nh.getParam("joints", joints_param))
  {
    ROS_ASSERT(joint_param.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    try
    {
      for (auto joint : joints_param)
      {
        if (joint.second.hasMember("servo_id"))
        {
          joint_name_.push_back(static_cast<std::string>(joint.first));
          servo_id_.push_back(static_cast<int>(joint.second["servo_id"]));
        }
        else
        {
          ROS_ERROR_STREAM("Skipping joint " << joint.first << " because it does not specify servo_id");
          continue;
        }
        if (joint.second.hasMember("gear_ratio"))
        {
          gear_ratio_.push_back(static_cast<double>(joint.second["gear_ratio"]));
        }
        else
        {
          gear_ratio_.push_back(1.0);
        }
        if (joint.second.hasMember("reverse"))
        {
          double direction = static_cast<bool>(joint.second["reverse"]) ? -1.0 : 1.0;
          direction_.push_back(direction);
        }
        else
        {
          direction_.push_back(1.0);
        }
      }
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_FATAL_STREAM("Error reading joint configuration: " << e.getMessage());
      return false;
    }
  }

  interface_.reset(servo_id_);
  ros::Duration(1.0).sleep();

  num_joints_ = joint_name_.size();
  position_.resize(num_joints_, 0.0);
  velocity_.resize(num_joints_, 0.0);
  effort_.resize(num_joints_, 0.0);
  command_.resize(num_joints_, 0.0);
  prev_command_.resize(num_joints_, 0.0);

  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    hardware_interface::JointStateHandle state_handle(joint_name_[i], &position_[i], &velocity_[i], &effort_[i]);
    joint_state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle pos_handle(state_handle, &command_[i]);
    position_joint_interface_.registerHandle(pos_handle);

    interface_.setServoMode(servo_id_[i], OPTIONS_RUN_FREE);
    interface_.setServoMode(servo_id_[i], OPTIONS_CONTROL_POSITION);
    interface_.setTrajectoryType(servo_id_[i], TRAJECTORY_NORMAL);
    interface_.setServoMode(servo_id_[i], OPTIONS_RUN_NORMAL);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  return true;
}

void B3MHwInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    short deg100 = interface_.getCurrentPosition(servo_id_[i]);
    double rad = (static_cast<double>(deg100) / 100.0) * (M_PI / 180.0);
    position_[i] = rad / gear_ratio_[i] * direction_[i];
  }
}

void B3MHwInterface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  std::vector<uint8_t> servo_ids;
  std::vector<short> target_positions;
  for (std::size_t i = 0; i < num_joints_; ++i)
  {
    if (std::abs(command_[i] - prev_command_[i]) > std::numeric_limits<double>::epsilon())
    {
      double rad = command_[i] * gear_ratio_[i] * direction_[i];
      short deg100 = static_cast<short>(rad * (180.0 / M_PI) * 100.0);
      target_positions.push_back(deg100);
      servo_ids.push_back(servo_id_[i]);
    }
    prev_command_[i] = command_[i];
  }
  interface_.setDesiredPosition(servo_ids, target_positions);
}

}  // namespace b3m_hw_interface

PLUGINLIB_EXPORT_CLASS(b3m_hw_interface::B3MHwInterface, hardware_interface::RobotHW)
