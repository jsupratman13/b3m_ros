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
#ifndef B3M_INTERFACE_HPP_
#define B3M_INTERFACE_HPP_

#include <vector>
#include "b3m_driver/b3m_map.hpp"
#include "b3m_driver/b3m_driver.hpp"

namespace b3m_driver
{
class B3MInterface
{
public:
  B3MInterface();
  ~B3MInterface();
  void connect(const std::string& port, uint32_t baudrate);
  void disconnect();
  bool load(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  void load(std::vector<uint8_t> servo_ids, uint8_t error_option = RETURN_ERROR_STATUS);
  bool save(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  void save(std::vector<uint8_t> servo_ids, uint8_t error_option = RETURN_ERROR_STATUS);
  void reset(uint8_t servo_id, uint8_t time = 100);
  void reset(std::vector<uint8_t> servo_ids, uint8_t time = 0);

  // system related functions
  bool setServoID(uint8_t servo_id, uint8_t new_servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setBaudrate(uint8_t servo_id, uint8_t baudrate, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPositionMin(uint8_t servo_id, short position, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPositionMax(uint8_t servo_id, short position, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPositionCenterOffset(uint8_t servo_id, short offset, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMCUTempLimit(uint8_t servo_id, short temperature, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMCUTempPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMotorTempLimit(uint8_t servo_id, short temperature, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMotorTempPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setCurrentLimit(uint8_t servo_id, unsigned short current, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setCurrentPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setLockDetectTime(uint8_t servo_id, uint8_t time, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setLockDetectOutRate(uint8_t servo_id, uint8_t rate, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setLockDetectPowerRatio(uint8_t servo_id, uint8_t ratio, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setInputVoltageMin(uint8_t servo_id, unsigned short voltage, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setInputVoltageMax(uint8_t servo_id, unsigned short voltage, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDeadbandWidth(uint8_t servo_id, unsigned short width, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMotorCWPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setMotorCCWPWMLimit(uint8_t servo_id, uint8_t pwm, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getServoID(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getBaudrate(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getPositionMin(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getPositionMax(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getPositionCenterOffset(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getMCUTempLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getMCUTempPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getMotorTempLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getMotorTempPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getCurrentLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getCurrentPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getLockDetectTime(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getLockDetectOutRate(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getLockDetectPowerRatio(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getInputVoltageMin(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getInputVoltageMax(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getDeadbandWidth(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getMotorCWPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getMotorCCWPWMLimit(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);

  // servo parameter related functions
  bool setServoOption(uint8_t servo_id, uint8_t option, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setServoMode(uint8_t servo_id, uint8_t mode, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setTrajectoryType(uint8_t servo_id, uint8_t type, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDesiredPosition(uint8_t servo_id, short position, unsigned short duration = 0,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions, unsigned short duration = 0,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDesiredVelocity(uint8_t servo_id, short velocity, uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredVelocity(std::vector<uint8_t> servo_ids, std::vector<short> velocities,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDesiredTime(uint8_t servo_id, unsigned short time, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDesiredTorque(uint8_t servo_id, short torque, uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredTorque(std::vector<uint8_t> servo_ids, std::vector<short> torques,
                        uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPWMFrequency(uint8_t servo_id, unsigned short frequency, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setEncoderCount(uint8_t servo_id, long value, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getServoOption(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getServoMode(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getTrajectoryType(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getDesiredPosition(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrentPosition(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getPreviousPosition(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getDesiredVelocity(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrentVelocity(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getPreviousVelocity(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getDesiredTime(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getRunningTime(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getWorkingTime(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getDesiredTorque(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getSystemClock(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getSamplingTime(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getMCUTemp(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getMotorTemp(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrent(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getInputVoltage(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getPWMCount(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getPWMFrequency(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getCurrentEncoderValue(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  long getTotalEncoderCount(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getHallICState(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);

  // control parameter related functions
  bool setPIDGainPresetNo(uint8_t servo_id, uint8_t no, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPGain0(uint8_t servo_id, unsigned long p_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setIGain0(uint8_t servo_id, unsigned long i_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDGain0(uint8_t servo_id, unsigned long d_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setStaticFriction0(uint8_t servo_id, unsigned short static_friction, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDynamicFriction0(uint8_t servo_id, unsigned short dynamic_friction,
                           uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPGain1(uint8_t servo_id, unsigned long p_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setIGain1(uint8_t servo_id, unsigned long i_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDGain1(uint8_t servo_id, unsigned long d_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setStaticFriction1(uint8_t servo_id, unsigned short static_friction, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDynamicFriction1(uint8_t servo_id, unsigned short dynamic_friction,
                           uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPGain2(uint8_t servo_id, unsigned long p_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setIGain2(uint8_t servo_id, unsigned long i_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDGain2(uint8_t servo_id, unsigned long d_gain, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setStaticFriction2(uint8_t servo_id, unsigned short static_friction, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setDynamicFriction2(uint8_t servo_id, unsigned short dynamic_friction,
                           uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getPIDGainPresetNo(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getPGain0(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getIGain0(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getDGain0(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getStaticFriction0(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getDynamicFriction0(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getPGain1(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getIGain1(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getDGain1(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getStaticFriction1(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getDynamicFriction1(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getPGain2(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getIGain2(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned long getDGain2(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getStaticFriction2(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  unsigned short getDynamicFriction2(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);

  // version information functions
  char getModelVoltageClass(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getModelVersion(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getModelTorqueNumber(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getModelCaseNumber(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  char getModelMotorType(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  char getModelDeviceType(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getFirmwareBuildNumber(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getFirmwareRevisionNumber(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getFirmwareMinorVersion(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  uint8_t getFirmwareMajorVersion(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);

private:
  B3MDriver b3m_driver_;

  std::string errorToString(uint8_t option, uint8_t error)
  {
    switch (option)
    {
      case RETURN_ERROR_STATUS:
        switch (error)
        {
          case SYSTEM_ERROR:
            return "SYSTEM_ERROR";
          case MOTOR_ERROR:
            return "MOTOR_ERROR";
          case UART_ERROR:
            return "UART_ERROR";
          case COMMAND_ERROR:
            return "COMMAND_ERROR";
          default:
            return "UNKNOWN ERROR: " + std::to_string(error);
        }
      case RETURN_SYSTEM_STATUS:
        switch (error)
        {
          case SYSTEM_ERROR_WATCHDOG:
            return "SYSTEM_ERROR_WATCHDOG";
          case SYSTEM_ERROR_FLASH_ADDRESS:
            return "SYSTEM_ERROR_FLASH_ADDRESS";
          case SYSTEM_ERROR_MEMORY_ALLOCATION:
            return "SYSTEM_ERROR_MEMORY_ALLOCATION";
          case SYSTEM_ERROR_INPUT_VOLTAGE:
            return "SYSTEM_ERROR_INPUT_VOLTAGE";
          case SYSTEM_ERROR_MOTOR_TEMP:
            return "SYSTEM_ERROR_MOTOR_TEMP";
          case SYSTEM_ERROR_AD_CONVERSION:
            return "SYSTEM_ERROR_AD_CONVERSION";
          case SYSTEM_ERROR_I2C:
            return "SYSTEM_ERROR_I2C";
          case SYSTEM_ERROR_SPI:
            return "SYSTEM_ERROR_SPI";
          default:
            return "UNKNOWN ERROR: " + std::to_string(error);
        }
      case RETURN_MOTOR_STATUS:
        switch (error)
        {
          case MOTOR_ERROR_EXCEED_MOTOR_TEMP:
            return "MOTOR_ERROR_EXCEED_MOTOR_TEMP";
          case MOTOR_ERROR_LOCK_DETECT:
            return "MOTOR_ERROR_LOCK_DETECT";
          case MOTOR_ERROR_EXCEED_CURRENT_LIMIT:
            return "MOTOR_ERROR_EXCEED_CURRENT_LIMIT";
          case MOTOR_ERROR_HALL_IC:
            return "MOTOR_ERROR_HALL_IC";
          default:
            return "UNKNOWN ERROR: " + std::to_string(error);
        }
      case RETURN_UART_STATUS:
        switch (error)
        {
          case UART_ERROR_FRAMING:
            return "UART_ERROR_FRAMING";
          case UART_ERROR_PARITY:
            return "UART_ERROR_PARITY";
          case UART_ERROR_BREAK:
            return "UART_ERROR_BREAK";
          case UART_ERROR_OVERRUN:
            return "UART_ERROR_OVERRUN";
          default:
            return "UNKNOWN ERROR: " + std::to_string(error);
        }
      case RETURN_COMMAND_STATUS:
        switch (error)
        {
          case COMMAND_ERROR_CHECKSUM:
            return "COMMAND_ERROR_CHECKSUM";
          case COMMAND_ERROR_LENGTH:
            return "COMMAND_ERROR_LENGTH";
          case COMMAND_ERROR_SIZE:
            return "COMMAND_ERROR_SIZE";
          case COMMAND_ERROR_ADDRESS:
            return "COMMAND_ERROR_ADDRESS";
          case COMMAND_ERROR_WRONG_COMMAND:
            return "COMMAND_ERROR_WRONG_COMMAND";
          default:
            return "UNKNOWN ERROR: " + std::to_string(error);
        }
      default:
        return "UNKNOWN OPTION: " + std::to_string(option);
    }
  }
};  // class B3MInterface

}  // namespace b3m_driver
#endif  // B3M_INTERFACE_HPP_
