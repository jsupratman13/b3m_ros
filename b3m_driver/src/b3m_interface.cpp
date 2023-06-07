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
  if (!b3m_driver_.send(COMMAND_TYPE_LOAD, RETURN_ERROR_STATUS, servo_id))
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to load" << std::endl;
}

void B3MInterface::load(std::vector<uint8_t> servo_ids)
{
  b3m_driver_.send(COMMAND_TYPE_LOAD, RETURN_ERROR_STATUS, std::move(servo_ids));
}

void B3MInterface::save(uint8_t servo_id)
{
  if (!b3m_driver_.send(COMMAND_TYPE_SAVE, RETURN_ERROR_STATUS, servo_id))
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to save" << std::endl;
}

void B3MInterface::save(std::vector<uint8_t> servo_ids)
{
  b3m_driver_.send(COMMAND_TYPE_SAVE, RETURN_ERROR_STATUS, std::move(servo_ids));
}

// system related functions
bool B3MInterface::setServoID(uint8_t servo_id, uint8_t new_servo_id)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, new_servo_id, SYSTEM_ID))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set new servo id" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setBaudrate(uint8_t servo_id, uint8_t baudrate)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, baudrate, SYSTEM_BAUDRATE))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set baudrate" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMin(uint8_t servo_id, short position)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, position, SYSTEM_POSITION_MIN))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position min" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionMax(uint8_t servo_id, short position)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, position, SYSTEM_POSITION_MAX))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position max" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPositionCenterOffset(uint8_t servo_id, short offset)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, offset, SYSTEM_POSITION_CENTER))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set position center offset" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempLimit(uint8_t servo_id, short temperature)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, temperature, SYSTEM_MCU_TEMP_LIMIT))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMCUTempPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_MCU_TEMP_LIMIT_PR))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set MCU temperature PWM limit"
              << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempLimit(uint8_t servo_id, short temperature)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, temperature, SYSTEM_MOTOR_TEMP_LIMIT))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorTempPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_MOTOR_TEMP_LIMIT_PR))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor temperature PWM limit"
              << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentLimit(uint8_t servo_id, unsigned short current)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, current, SYSTEM_CURRENT_LIMIT))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setCurrentPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_CURRENT_LIMIT_PR))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set current PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectTime(uint8_t servo_id, uint8_t time)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, time, SYSTEM_LOCKDETECT_TIME))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect time" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectOutRate(uint8_t servo_id, uint8_t rate)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, rate, SYSTEM_LOCKDETECT_OUTRATE))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect out rate" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setLockDetectPowerRatio(uint8_t servo_id, uint8_t ratio)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, ratio, SYSTEM_LOCKDETECT_TIME_PR))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set lock detect power ratio" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMin(uint8_t servo_id, unsigned short voltage)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, voltage, SYSTEM_INPUT_VOLTAGE_MIN))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set input voltage min" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setInputVoltageMax(uint8_t servo_id, unsigned short voltage)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, voltage, SYSTEM_INPUT_VOLTAGE_MAX))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set input voltage max" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_TORQUE_LIMIT))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDeadbandWidth(uint8_t servo_id, unsigned short width)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, width, SYSTEM_DEADBAND_WIDTH))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set deadband width" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCWPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_MOTOR_CW_RATIO))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CW PWM limit" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setMotorCCWPWMLimit(uint8_t servo_id, uint8_t pwm)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, pwm, SYSTEM_MOTOR_CCW_RATIO))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set motor CCW PWM limit" << std::endl;
    return false;
  }
  return true;
}

// TODO: system related read functions

// servo parameter related functions
bool B3MInterface::setServoOption(uint8_t servo_id, uint8_t option)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, option, SERVO_SERVO_OPTION))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo option" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setServoMode(uint8_t servo_id, uint8_t mode)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, mode, SERVO_SERVO_MODE))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set servo mode" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setTrajectoryType(uint8_t servo_id, uint8_t type)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, type, SERVO_RUN_MODE))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set trajectory type" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredPosition(uint8_t servo_id, short position)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, position, SERVO_DESIRED_POSITION))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired position" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredVelocity(uint8_t servo_id, short velocity)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, velocity, SERVO_DESIRED_VELOCITY))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired velocity" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredTime(uint8_t servo_id, unsigned short time)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, time, SERVO_DESIRED_TIME))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired time" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDesiredTorque(uint8_t servo_id, short torque)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, torque, SERVO_DESIRED_TORQUE))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set desired torque" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPWMFrequency(uint8_t servo_id, unsigned short frequency)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, frequency, SERVO_PWM_FREQUENCY))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PWM frequency" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setEncoderCount(uint8_t servo_id, long value)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, value, SERVO_ENCODER_COUNT))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set encoder count" << std::endl;
    return false;
  }
  return true;
}

// TODO: servo parameter related read functions

// control related functions
bool B3MInterface::setPIDGainPresetNo(uint8_t servo_id, uint8_t no)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, no, CONTROL_GAIN_PRESETNO))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set PID gain preset no" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain0(uint8_t servo_id, unsigned long p_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, p_gain, CONTROL_KP0))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain0(uint8_t servo_id, unsigned long i_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, i_gain, CONTROL_KI0))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain0(uint8_t servo_id, unsigned long d_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, d_gain, CONTROL_KD0))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction0(uint8_t servo_id, unsigned short static_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, static_friction, CONTROL_STATIC_FRICTION0))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction0(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, dynamic_friction, CONTROL_DYNAMIC_FRICTION0))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 0" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain1(uint8_t servo_id, unsigned long p_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, p_gain, CONTROL_KP1))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain1(uint8_t servo_id, unsigned long i_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, i_gain, CONTROL_KI1))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain1(uint8_t servo_id, unsigned long d_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, d_gain, CONTROL_KD1))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction1(uint8_t servo_id, unsigned short static_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, static_friction, CONTROL_STATIC_FRICTION1))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction1(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, dynamic_friction, CONTROL_DYNAMIC_FRICTION1))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 1" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setPGain2(uint8_t servo_id, unsigned long p_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, p_gain, CONTROL_KP2))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set P gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setIGain2(uint8_t servo_id, unsigned long i_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, i_gain, CONTROL_KI2))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set I gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDGain2(uint8_t servo_id, unsigned long d_gain)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, d_gain, CONTROL_KD2))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set D gain 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setStaticFriction2(uint8_t servo_id, unsigned short static_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, static_friction, CONTROL_STATIC_FRICTION2))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set static friction 2" << std::endl;
    return false;
  }
  return true;
}

bool B3MInterface::setDynamicFriction2(uint8_t servo_id, unsigned short dynamic_friction)
{
  if (!b3m_driver_.send(COMMAND_TYPE_WRITE, RETURN_ERROR_STATUS, servo_id, dynamic_friction, CONTROL_DYNAMIC_FRICTION2))
  {
    std::cout << "Servo: " << static_cast<unsigned>(servo_id) << " failed to set dynamic friction 2" << std::endl;
    return false;
  }
  return true;
}
