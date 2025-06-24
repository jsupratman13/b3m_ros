/*********************************************************************
 * Copyright (c) 2025
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

#include "b3m_driver/b3m_async_interface.hpp"
#include "b3m_driver/b3m_map.hpp"

namespace b3m_driver
{

B3MAsyncInterface::B3MAsyncInterface() : driver_()
{
}
B3MAsyncInterface::~B3MAsyncInterface()
{
  disconnect();
}

void B3MAsyncInterface::connect(const std::string& port, unsigned int baudrate)
{
  if (!driver_.open(port, baudrate))
  {
    throw std::runtime_error("Failed to open serial port: " + port);
  }
}

void B3MAsyncInterface::disconnect()
{
  driver_.close();
}

void B3MAsyncInterface::reset(std::vector<uint8_t> servo_ids, uint8_t time)
{
  driver_.reset(std::move(servo_ids), time);
}

bool B3MAsyncInterface::setServoMode(uint8_t servo_id, uint8_t mode, uint8_t error_option)
{
  uint8_t error = driver_.write(servo_id, error_option, mode, SERVO_SERVO_MODE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo mode\n";
    return false;
  }
  return true;
}

bool B3MAsyncInterface::setTrajectoryType(uint8_t servo_id, uint8_t type, uint8_t error_option)
{
  uint8_t error = driver_.write(servo_id, error_option, type, SERVO_RUN_MODE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set trajectory type\n";
    return false;
  }
  return true;
}

bool B3MAsyncInterface::setPIDGainPresetNo(uint8_t servo_id, uint8_t no, uint8_t error_option)
{
  uint8_t error = driver_.write(servo_id, error_option, no, CONTROL_GAIN_PRESETNO);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PID gain preset number\n";
    return false;
  }
  return true;
}

bool B3MAsyncInterface::setDesiredPosition(uint8_t servo_id, short position, unsigned short duration,
                                           uint8_t error_option)
{
  uint8_t error = driver_.setPosition(servo_id, error_option, position, duration);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired position\n";
    return false;
  }
  return true;
}

void B3MAsyncInterface::setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions,
                                           unsigned short duration, uint8_t error_option)
{
  driver_.setPosition(std::move(servo_ids), error_option, std::move(positions), duration);
}

short B3MAsyncInterface::getCurrentPosition(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = driver_.read(servo_id, error_option, SERVO_CURRENT_POSITION, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current position\n";
    return 0;
  }
  return data;
}
short B3MAsyncInterface::getCurrentVelocity(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = driver_.read(servo_id, error_option, SERVO_CURRENT_VELOCITY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current velocity\n";
    return 0;
  }
  return data;
}
}  // namespace b3m_driver
