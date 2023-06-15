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
#include "b3m_driver/b3m_map.hpp"
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

void B3MInterface::load(uint8_t servo_id)
{
  if (b3m_driver_.load(servo_id, RETURN_ERROR_STATUS))
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to load" << std::endl;
}

void B3MInterface::load(std::vector<uint8_t> servo_ids)
{
  b3m_driver_.load(std::move(servo_ids), RETURN_ERROR_STATUS);
}

void B3MInterface::save(uint8_t servo_id)
{
  if (b3m_driver_.save(servo_id, RETURN_ERROR_STATUS))
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to save" << std::endl;
}

void B3MInterface::save(std::vector<uint8_t> servo_ids)
{
  b3m_driver_.save(std::move(servo_ids), RETURN_ERROR_STATUS);
}

void B3MInterface::reset(uint8_t servo_id)
{
  b3m_driver_.reset(servo_id, 100);
}

// system related functions
bool B3MInterface::setServoID(uint8_t servo_id, uint8_t new_servo_id)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, new_servo_id, SYSTEM_ID))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set new servo id" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setBaudrate(uint8_t servo_id, uint8_t baudrate)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, baudrate, SYSTEM_BAUDRATE))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set baudrate" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMin(uint8_t servo_id, short position)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, position, SYSTEM_POSITION_MIN))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position min" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMax(uint8_t servo_id, short position)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, position, SYSTEM_POSITION_MAX))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position max" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionCenterOffset(uint8_t servo_id, short offset)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, offset, SYSTEM_POSITION_CENTER))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position center offset" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempLimit(uint8_t servo_id, short temperature)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, temperature, SYSTEM_MCU_TEMP_LIMIT))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_MCU_TEMP_LIMIT_PR))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature PWM limit"
              << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempLimit(uint8_t servo_id, short temperature)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, temperature, SYSTEM_MOTOR_TEMP_LIMIT))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_MOTOR_TEMP_LIMIT_PR))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature PWM limit"
              << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentLimit(uint8_t servo_id, unsigned short current)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, current, SYSTEM_CURRENT_LIMIT))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_CURRENT_LIMIT_PR))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectTime(uint8_t servo_id, uint8_t time)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, time, SYSTEM_LOCKDETECT_TIME))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect time" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectOutRate(uint8_t servo_id, uint8_t rate)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, rate, SYSTEM_LOCKDETECT_OUTRATE))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect out rate" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectPowerRatio(uint8_t servo_id, uint8_t ratio)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, ratio, SYSTEM_LOCKDETECT_TIME_PR))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect power ratio" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMin(uint8_t servo_id, unsigned short voltage)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, voltage, SYSTEM_INPUT_VOLTAGE_MIN))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set input voltage min" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMax(uint8_t servo_id, unsigned short voltage)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, voltage, SYSTEM_INPUT_VOLTAGE_MAX))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set input voltage max" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_TORQUE_LIMIT))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDeadbandWidth(uint8_t servo_id, unsigned short width)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, width, SYSTEM_DEADBAND_WIDTH))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set deadband width" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCWPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_MOTOR_CW_RATIO))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CW PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCCWPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, pwm, SYSTEM_MOTOR_CCW_RATIO))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CCW PWM limit" << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getServoID(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_ID, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo ID" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getBaudrate(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_BAUDRATE, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get baudrate" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionMin(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_POSITION_MIN, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get position min" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionMax(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_POSITION_MAX, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get position max" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPositionCenterOffset(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_POSITION_CENTER, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get position center offset" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMCUTempLimit(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MCU_TEMP_LIMIT, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temp" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMCUTempPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MCU_TEMP_LIMIT_PR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temp PWM limit" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMotorTempLimit(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MOTOR_TEMP_LIMIT, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temp limit" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorTempPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MOTOR_TEMP_LIMIT_PR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temp PWM limit" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getCurrentLimit(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_CURRENT_LIMIT, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current limit" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getCurrentPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_CURRENT_LIMIT_PR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current PWM limit" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectTime(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_LOCKDETECT_TIME, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detected" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectOutRate(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_LOCKDETECT_OUTRATE, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detect out rate" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getLockDetectPowerRatio(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_LOCKDETECT_TIME_PR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get lock detect power ratio" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltageMin(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_INPUT_VOLTAGE_MIN, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get input voltage min" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltageMax(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_INPUT_VOLTAGE_MAX, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get input voltage max" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_TORQUE_LIMIT, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM limit" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDeadbandWidth(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_DEADBAND_WIDTH, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get deadband width" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorCWPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MOTOR_CW_RATIO, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor CW PWM limit" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getMotorCCWPWMLimit(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SYSTEM_MOTOR_CCW_RATIO, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor CCW PWM limit" << std::endl;
    return 0;
  }
  return data;
}

// servo parameter related functions
bool B3MInterface::setServoOption(uint8_t servo_id, uint8_t option)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, option, SERVO_SERVO_OPTION))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo option" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setServoMode(uint8_t servo_id, uint8_t mode)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, mode, SERVO_SERVO_MODE) != 0x00)
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo mode" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setTrajectoryType(uint8_t servo_id, uint8_t type)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, type, SERVO_RUN_MODE))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set trajectory type" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredPosition(uint8_t servo_id, short position)
{
  if (b3m_driver_.setPosition(servo_id, RETURN_ERROR_STATUS, position, 0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired position" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredVelocity(uint8_t servo_id, short velocity)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, velocity, SERVO_DESIRED_VELOCITY))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired velocity" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredTime(uint8_t servo_id, unsigned short time)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, time, SERVO_DESIRED_TIME))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired time" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredTorque(uint8_t servo_id, short torque)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, torque, SERVO_DESIRED_TORQUE))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired torque" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPWMFrequency(uint8_t servo_id, unsigned short frequency)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, frequency, SERVO_PWM_FREQUENCY))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM frequency" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setEncoderCount(uint8_t servo_id, long value)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, value, SERVO_ENCODER_COUNT))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set encoder count" << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getServoOption(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_SERVO_OPTION, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo option" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getServoMode(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_SERVO_MODE, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get servo mode" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getTrajectoryType(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_RUN_MODE, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get trajectory type" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredPosition(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_DESIRED_POSITION, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired position" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrentPosition(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_CURRENT_POSITION, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current position" << std::endl;
    return data;
  }
  return data;
}

short B3MInterface::getPreviousPosition(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_PREVIOUS_POSITION, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get previous position" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredVelocity(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_DESIRED_VELOCITY, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired velocity" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrentVelocity(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_CURRENT_VELOCITY, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current velocity" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getPreviousVelocity(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_PREVIOUS_VELOCITY, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get previous velocity" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDesiredTime(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_DESIRED_TIME, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired time" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getRunningTime(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_RUNNING_TIME, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get running time" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getWorkingTime(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_WORKING_TIME, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get working time" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getDesiredTorque(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_DESIRED_TORQUE, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get desired torque" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getSystemClock(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_SYSTEM_CLOCK, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get system clock" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getSamplingTime(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_SAMPLING_TIME, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get sampling time" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMCUTemp(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_MCU_TEMP, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get MCU temp" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getMotorTemp(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_MOTOR_TEMP, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get motor temp" << std::endl;
    return 0;
  }
  return data;
}

short B3MInterface::getCurrent(uint8_t servo_id)
{
  short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_CURRENT, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getInputVoltage(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_INPUT_VOLTAGE, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get input voltage" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getPWMCount(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_PWM_DUTY, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM count" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getPWMFrequency(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_PWM_FREQUENCY, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get PWM frequency" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getCurrentEncoderValue(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_ENCODER_VALUE, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get current encoder value" << std::endl;
    return 0;
  }
  return data;
}

long B3MInterface::getTotalEncoderCount(uint8_t servo_id)
{
  long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_ENCODER_COUNT, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get total encoder count" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getHallICState(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, SERVO_HALLIC_STATE, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get hall IC state" << std::endl;
    return 0;
  }
  return data;
}

// control related functions
bool B3MInterface::setPIDGainPresetNo(uint8_t servo_id, uint8_t no)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, no, CONTROL_GAIN_PRESETNO))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PID gain preset no" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain0(uint8_t servo_id, unsigned long p_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, p_gain, CONTROL_KP0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain0(uint8_t servo_id, unsigned long i_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, i_gain, CONTROL_KI0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain0(uint8_t servo_id, unsigned long d_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, d_gain, CONTROL_KD0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction0(uint8_t servo_id, unsigned short static_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, static_friction, CONTROL_STATIC_FRICTION0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction0(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, dynamic_friction, CONTROL_DYNAMIC_FRICTION0))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain1(uint8_t servo_id, unsigned long p_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, p_gain, CONTROL_KP1))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain1(uint8_t servo_id, unsigned long i_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, i_gain, CONTROL_KI1))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain1(uint8_t servo_id, unsigned long d_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, d_gain, CONTROL_KD1))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction1(uint8_t servo_id, unsigned short static_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, static_friction, CONTROL_STATIC_FRICTION1))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction1(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, dynamic_friction, CONTROL_DYNAMIC_FRICTION1))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain2(uint8_t servo_id, unsigned long p_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, p_gain, CONTROL_KP2))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain2(uint8_t servo_id, unsigned long i_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, i_gain, CONTROL_KI2))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain2(uint8_t servo_id, unsigned long d_gain)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, d_gain, CONTROL_KD2))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction2(uint8_t servo_id, unsigned short static_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, static_friction, CONTROL_STATIC_FRICTION2))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction2(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (b3m_driver_.write(servo_id, RETURN_ERROR_STATUS, dynamic_friction, CONTROL_DYNAMIC_FRICTION2))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 2" << std::endl;
    return false;
  }
  return true;
}

uint8_t B3MInterface::getPIDGainPresetNo(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_GAIN_PRESETNO, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get hall IC state" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain0(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KP0, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain 0" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain0(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KI0, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain 0" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain0(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KD0, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain 0" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction0(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_STATIC_FRICTION0, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction 0" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction0(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_DYNAMIC_FRICTION0, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction 0" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain1(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KP1, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain 1" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain1(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KI1, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain 1" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain1(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KD1, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain 1" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction1(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_STATIC_FRICTION1, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction 1" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction1(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_DYNAMIC_FRICTION1, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction 1" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getPGain2(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KP2, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get P gain 2" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getIGain2(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KI2, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get I gain 2" << std::endl;
    return 0;
  }
  return data;
}

unsigned long B3MInterface::getDGain2(uint8_t servo_id)
{
  unsigned long data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_KD2, 4, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get D gain 2" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getStaticFriction2(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_STATIC_FRICTION2, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get static friction 2" << std::endl;
    return 0;
  }
  return data;
}

unsigned short B3MInterface::getDynamicFriction2(uint8_t servo_id)
{
  unsigned short data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONTROL_DYNAMIC_FRICTION2, 2, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get dynamic friction 2" << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelVoltageClass(uint8_t servo_id)
{
  char data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_MODEL_NUMBER_VOLTAGE_CLASS, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model voltage class" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getModelVersion(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_MODEL_NUMBER_VERSION, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model version" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getModelCaseNumber(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_MODEL_NUMBER_CASE, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model case number" << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelMotorType(uint8_t servo_id)
{
  char data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_MODEL_TYPE_MOTOR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model motor type" << std::endl;
    return 0;
  }
  return data;
}

char B3MInterface::getModelDeviceType(uint8_t servo_id)
{
  char data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_MODEL_TYPE_DEVICE, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get model device type" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareBuildNumber(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_FW_BUILD, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware build number" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareRevisionNumber(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_FW_REVISION, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware revision number" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareMinorVersion(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_FW_MINOR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware minor version" << std::endl;
    return 0;
  }
  return data;
}

uint8_t B3MInterface::getFirmwareMajorVersion(uint8_t servo_id)
{
  uint8_t data;
  if (b3m_driver_.read(servo_id, RETURN_ERROR_STATUS, CONFIG_FW_MAJOR, 1, data))
  {
    std::cerr << "Servo: " << static_cast<unsigned>(servo_id) << " failed to get firmware major version" << std::endl;
    return 0;
  }
  return data;
}
}  // namespace b3m_driver
