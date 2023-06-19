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

#include <iostream>
#include "b3m_driver/b3m_interface.hpp"

namespace b3m_driver
{
B3MInterface::B3MInterface()
{
}

B3MInterface::~B3MInterface()
{
  disconnect();
}

void B3MInterface::connect(const std::string& port, uint32_t baudrate)
{
  b3m_driver_.open(port, baudrate);
}

void B3MInterface::disconnect()
{
  b3m_driver_.close();
}

bool B3MInterface::load(uint8_t servo_id, uint8_t error_option)
{
  uint8_t error = b3m_driver_.load(servo_id, error_option);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to load due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

void B3MInterface::load(std::vector<uint8_t> servo_ids, uint8_t error_option)
{
  b3m_driver_.load(std::move(servo_ids), error_option);
}

bool B3MInterface::save(uint8_t servo_id, uint8_t error_option)
{
  uint8_t error = b3m_driver_.save(servo_id, error_option);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to save due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

void B3MInterface::save(std::vector<uint8_t> servo_ids, uint8_t error_option)
{
  b3m_driver_.save(std::move(servo_ids), error_option);
}

void B3MInterface::reset(uint8_t servo_id, uint8_t time)
{
  b3m_driver_.reset(servo_id, time);
}

void B3MInterface::reset(std::vector<uint8_t> servo_id, uint8_t time)
{
  b3m_driver_.reset(std::move(servo_id), time);
}

// system related functions
bool B3MInterface::setServoID(uint8_t servo_id, uint8_t new_servo_id, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, new_servo_id, SYSTEM_ID);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set new servo id due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setBaudrate(uint8_t servo_id, uint8_t baudrate, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, baudrate, SYSTEM_BAUDRATE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set baudrate due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMin(uint8_t servo_id, short position, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, position, SYSTEM_POSITION_MIN);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set min position due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMax(uint8_t servo_id, short position, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, position, SYSTEM_POSITION_MAX);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set max position due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionCenterOffset(uint8_t servo_id, short offset, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, offset, SYSTEM_POSITION_CENTER);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set center position offset due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempLimit(uint8_t servo_id, short temperature, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, temperature, SYSTEM_MCU_TEMP_LIMIT);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_MCU_TEMP_LIMIT_PR);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempLimit(uint8_t servo_id, short temperature, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, temperature, SYSTEM_MOTOR_TEMP_LIMIT);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_MOTOR_TEMP_LIMIT_PR);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentLimit(uint8_t servo_id, unsigned short current, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, current, SYSTEM_CURRENT_LIMIT);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_CURRENT_LIMIT_PR);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectTime(uint8_t servo_id, uint8_t time, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, time, SYSTEM_LOCKDETECT_TIME);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect time due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectOutRate(uint8_t servo_id, uint8_t rate, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, rate, SYSTEM_LOCKDETECT_OUTRATE);
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect out rate due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectPowerRatio(uint8_t servo_id, uint8_t ratio, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, ratio, SYSTEM_LOCKDETECT_TIME_PR);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect power ratio due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMin(uint8_t servo_id, unsigned short voltage, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, voltage, SYSTEM_INPUT_VOLTAGE_MIN);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set min input voltage due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMax(uint8_t servo_id, unsigned short voltage, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, voltage, SYSTEM_INPUT_VOLTAGE_MAX);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set max input voltage due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_TORQUE_LIMIT);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDeadbandWidth(uint8_t servo_id, unsigned short width, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, width, SYSTEM_DEADBAND_WIDTH);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set deadband width due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCWPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_MOTOR_CW_RATIO);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CW PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCCWPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, pwm, SYSTEM_MOTOR_CCW_RATIO);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CCW PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getServoID(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_ID, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo ID due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getBaudrate(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_BAUDRATE, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get baudrate due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionMin(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_POSITION_MIN, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get min position due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionMax(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_POSITION_MAX, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get max position due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionCenterOffset(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_POSITION_CENTER, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get center position offset due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMCUTempLimit(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MCU_TEMP_LIMIT, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temperature due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMCUTempPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MCU_TEMP_LIMIT_PR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temperature PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMotorTempLimit(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MOTOR_TEMP_LIMIT, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temperature limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorTempPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MOTOR_TEMP_LIMIT_PR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temperature PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getCurrentLimit(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_CURRENT_LIMIT, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getCurrentPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_CURRENT_LIMIT_PR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectTime(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_LOCKDETECT_TIME, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detect time due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectOutRate(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_LOCKDETECT_OUTRATE, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detect out rate due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectPowerRatio(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_LOCKDETECT_TIME_PR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detect power ratio due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltageMin(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_INPUT_VOLTAGE_MIN, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get min input voltage due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltageMax(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_INPUT_VOLTAGE_MAX, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get max input voltage due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_TORQUE_LIMIT, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDeadbandWidth(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_DEADBAND_WIDTH, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get deadband width due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorCWPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MOTOR_CW_RATIO, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor CW PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorCCWPWMLimit(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SYSTEM_MOTOR_CCW_RATIO, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor CCW PWM limit due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

// servo parameter related functions
bool B3MInterface::setServoOption(uint8_t servo_id, uint8_t option, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, option, SERVO_SERVO_OPTION);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo option due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setServoMode(uint8_t servo_id, uint8_t mode, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, mode, SERVO_SERVO_MODE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo mode due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setTrajectoryType(uint8_t servo_id, uint8_t type, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, type, SERVO_RUN_MODE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set trajectory type due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredPosition(uint8_t servo_id, short position, unsigned short duration, uint8_t error_option)
{
  uint8_t error = b3m_driver_.setPosition(servo_id, error_option, position, duration);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired position due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

void B3MInterface::setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions,
                                      unsigned short duration, uint8_t error_option)
{
  b3m_driver_.setPosition(std::move(servo_ids), error_option, std::move(positions), duration);
}

bool B3MInterface::setDesiredVelocity(uint8_t servo_id, short velocity, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, velocity, SERVO_DESIRED_VELOCITY);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired velocity due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

void B3MInterface::setDesiredVelocity(std::vector<uint8_t> servo_ids, std::vector<short> velocities,
                                      uint8_t error_option)
{
  b3m_driver_.write(std::move(servo_ids), error_option, std::move(velocities), SERVO_DESIRED_VELOCITY);
}

bool B3MInterface::setDesiredTime(uint8_t servo_id, unsigned short time, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, time, SERVO_DESIRED_TIME);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired time due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredTorque(uint8_t servo_id, short torque, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, torque, SERVO_DESIRED_TORQUE);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired torque due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

void B3MInterface::setDesiredTorque(std::vector<uint8_t> servo_ids, std::vector<short> torques, uint8_t error_option)
{
  b3m_driver_.write(std::move(servo_ids), error_option, std::move(torques), SERVO_DESIRED_TORQUE);
}

bool B3MInterface::setPWMFrequency(uint8_t servo_id, unsigned short frequency, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, frequency, SERVO_PWM_FREQUENCY);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM frequency due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setEncoderCount(uint8_t servo_id, long value, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, value, SERVO_ENCODER_COUNT);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set encoder count due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getServoOption(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_SERVO_OPTION, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo option due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getServoMode(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_SERVO_MODE, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo mode due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getTrajectoryType(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_RUN_MODE, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get trajectory type due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredPosition(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_DESIRED_POSITION, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired position due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrentPosition(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_CURRENT_POSITION, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current position due to "
              << errorToString(error_option, error) << std::endl;
    return data;
  }
  return data;
}

short B3MInterface::getPreviousPosition(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_PREVIOUS_POSITION, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get previous position due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredVelocity(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_DESIRED_VELOCITY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired velocity due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrentVelocity(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_CURRENT_VELOCITY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current velocity due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPreviousVelocity(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_PREVIOUS_VELOCITY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get previous velocity due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDesiredTime(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_DESIRED_TIME, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired time due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getRunningTime(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_RUNNING_TIME, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get running time due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getWorkingTime(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_WORKING_TIME, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get working time due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredTorque(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_DESIRED_TORQUE, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired torque due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getSystemClock(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_SYSTEM_CLOCK, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get system clock due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getSamplingTime(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_SAMPLING_TIME, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get sampling time due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMCUTemp(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_MCU_TEMP, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temperature due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMotorTemp(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_MOTOR_TEMP, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temperature due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrent(uint8_t servo_id, uint8_t error_option)
{
  short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_CURRENT, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltage(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_INPUT_VOLTAGE, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get input voltage due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getPWMCount(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_PWM_DUTY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM count due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getPWMFrequency(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_PWM_FREQUENCY, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM frequency due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getCurrentEncoderValue(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_ENCODER_VALUE, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current encoder value due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

long B3MInterface::getTotalEncoderCount(uint8_t servo_id, uint8_t error_option)
{
  long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_ENCODER_COUNT, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get total encoder count due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getHallICState(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, SERVO_HALLIC_STATE, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get hall IC state due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

// control related functions
bool B3MInterface::setPIDGainPresetNo(uint8_t servo_id, uint8_t no, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, no, CONTROL_GAIN_PRESETNO);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PID gain preset no. due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain0(uint8_t servo_id, unsigned long p_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, p_gain, CONTROL_KP0);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain0(uint8_t servo_id, unsigned long i_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, i_gain, CONTROL_KI0);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain0(uint8_t servo_id, unsigned long d_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, d_gain, CONTROL_KD0);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction0(uint8_t servo_id, unsigned short static_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, static_friction, CONTROL_STATIC_FRICTION0);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction0(uint8_t servo_id, unsigned short dynamic_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, dynamic_friction, CONTROL_DYNAMIC_FRICTION0);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain1(uint8_t servo_id, unsigned long p_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, p_gain, CONTROL_KP1);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain1(uint8_t servo_id, unsigned long i_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, i_gain, CONTROL_KI1);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain1(uint8_t servo_id, unsigned long d_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, d_gain, CONTROL_KD1);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction1(uint8_t servo_id, unsigned short static_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, static_friction, CONTROL_STATIC_FRICTION1);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction1(uint8_t servo_id, unsigned short dynamic_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, dynamic_friction, CONTROL_DYNAMIC_FRICTION1);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain2(uint8_t servo_id, unsigned long p_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, p_gain, CONTROL_KP2);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain2(uint8_t servo_id, unsigned long i_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, i_gain, CONTROL_KI2);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain2(uint8_t servo_id, unsigned long d_gain, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, d_gain, CONTROL_KD2);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction2(uint8_t servo_id, unsigned short static_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, static_friction, CONTROL_STATIC_FRICTION2);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction2(uint8_t servo_id, unsigned short dynamic_friction, uint8_t error_option)
{
  uint8_t error = b3m_driver_.write(servo_id, error_option, dynamic_friction, CONTROL_DYNAMIC_FRICTION2);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getPIDGainPresetNo(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_GAIN_PRESETNO, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PID gain preset no. due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain0(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KP0, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain0(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KI0, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain0(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KD0, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction0(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_STATIC_FRICTION0, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction0(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_DYNAMIC_FRICTION0, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction no. 0 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain1(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KP1, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain1(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KI1, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain1(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KD1, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction1(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_STATIC_FRICTION1, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction1(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_DYNAMIC_FRICTION1, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction no. 1 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain2(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KP2, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain2(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KI2, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain2(uint8_t servo_id, uint8_t error_option)
{
  unsigned long data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_KD2, 4, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction2(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_STATIC_FRICTION2, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction2(uint8_t servo_id, uint8_t error_option)
{
  unsigned short data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONTROL_DYNAMIC_FRICTION2, 2, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction no. 2 due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelVoltageClass(uint8_t servo_id, uint8_t error_option)
{
  char data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_MODEL_NUMBER_VOLTAGE_CLASS, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model voltage class due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getModelVersion(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_MODEL_NUMBER_VERSION, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model version due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getModelCaseNumber(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_MODEL_NUMBER_CASE, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model case number due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelMotorType(uint8_t servo_id, uint8_t error_option)
{
  char data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_MODEL_TYPE_MOTOR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model motor type due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelDeviceType(uint8_t servo_id, uint8_t error_option)
{
  char data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_MODEL_TYPE_DEVICE, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model device type due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareBuildNumber(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_FW_BUILD, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware build number due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareRevisionNumber(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_FW_REVISION, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware revision number due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareMinorVersion(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_FW_MINOR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware minor version due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareMajorVersion(uint8_t servo_id, uint8_t error_option)
{
  uint8_t data;
  uint8_t error = b3m_driver_.read(servo_id, error_option, CONFIG_FW_MAJOR, 1, data);
  if (error)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware major version due to "
              << errorToString(error_option, error) << std::endl;
    return 0;
  }
  return data;
}
}  // namespace b3m_driver
