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

#include <cstdint>
#include "b3m_driver/b3m_driver.hpp"

namespace b3m_driver
{
class B3MInterface
{
private:
  B3MDriver b3m_driver_;

public:
  B3MInterface();
  ~B3MInterface();
  void connect(const std::string& port, uint32_t baudrate);
  void disconnect();
  void load(uint8_t servo_id);
  void load(std::vector<uint8_t> servo_ids);
  void save(uint8_t servo_id);
  void save(std::vector<uint8_t> servo_ids);
  void reset(uint8_t servo_id);
  void reset(std::vector<uint8_t> servo_ids);

  // system related functions
  bool setServoID(uint8_t servo_id, uint8_t new_servo_id);
  bool setBaudrate(uint8_t servo_id, uint8_t baudrate);
  bool setPositionMin(uint8_t servo_id, short position);
  bool setPositionMax(uint8_t servo_id, short position);
  bool setPositionCenterOffset(uint8_t servo_id, short offset);
  bool setMCUTempLimit(uint8_t servo_id, short temperature);
  bool setMCUTempPWMLimit(uint8_t servo_id, uint8_t pwm);
  bool setMotorTempLimit(uint8_t servo_id, short temperature);
  bool setMotorTempPWMLimit(uint8_t servo_id, uint8_t pwm);
  bool setCurrentLimit(uint8_t servo_id, unsigned short current);
  bool setCurrentPWMLimit(uint8_t servo_id, uint8_t pwm);
  bool setLockDetectTime(uint8_t servo_id, uint8_t time);
  bool setLockDetectOutRate(uint8_t servo_id, uint8_t rate);
  bool setLockDetectPowerRatio(uint8_t servo_id, uint8_t ratio);
  bool setInputVoltageMin(uint8_t servo_id, unsigned short voltage);
  bool setInputVoltageMax(uint8_t servo_id, unsigned short voltage);
  bool setPWMLimit(uint8_t servo_id, uint8_t pwm);
  bool setDeadbandWidth(uint8_t servo_id, unsigned short width);
  bool setMotorCWPWMLimit(uint8_t servo_id, uint8_t pwm);
  bool setMotorCCWPWMLimit(uint8_t servo_id, uint8_t pwm);
  uint8_t getServoID(uint8_t servo_id);
  unsigned long getBaudrate(uint8_t servo_id);
  short getPositionMin(uint8_t servo_id);
  short getPositionMax(uint8_t servo_id);
  short getPositionCenterOffset(uint8_t servo_id);
  short getMCUTempLimit(uint8_t servo_id);
  uint8_t getMCUTempPWMLimit(uint8_t servo_id);
  short getMotorTempLimit(uint8_t servo_id);
  uint8_t getMotorTempPWMLimit(uint8_t servo_id);
  unsigned short getCurrentLimit(uint8_t servo_id);
  uint8_t getCurrentPWMLimit(uint8_t servo_id);
  uint8_t getLockDetectTime(uint8_t servo_id);
  uint8_t getLockDetectOutRate(uint8_t servo_id);
  uint8_t getLockDetectPowerRatio(uint8_t servo_id);
  unsigned short getInputVoltageMin(uint8_t servo_id);
  unsigned short getInputVoltageMax(uint8_t servo_id);
  uint8_t getPWMLimit(uint8_t servo_id);
  unsigned short getDeadbandWidth(uint8_t servo_id);
  uint8_t getMotorCWPWMLimit(uint8_t servo_id);
  uint8_t getMotorCCWPWMLimit(uint8_t servo_id);

  // servo parameter related functions
  bool setServoOption(uint8_t servo_id, uint8_t option);
  bool setServoMode(uint8_t servo_id, uint8_t mode);
  bool setTrajectoryType(uint8_t servo_id, uint8_t type);
  bool setDesiredPosition(uint8_t servo_id, short position);
  bool setDesiredVelocity(uint8_t servo_id, short velocity);
  bool setDesiredTime(uint8_t servo_id, unsigned short time);
  bool setDesiredTorque(uint8_t servo_id, short torque);
  bool setPWMFrequency(uint8_t servo_id, unsigned short frequency);
  bool setEncoderCount(uint8_t servo_id, long value);
  uint8_t getServoOption(uint8_t servo_id);
  unsigned short getServoMode(uint8_t servo_id);
  uint8_t getTrajectoryType(uint8_t servo_id);
  short getDesiredPosition(uint8_t servo_id);
  short getCurrentPosition(uint8_t servo_id);
  short getPreviousPosition(uint8_t servo_id);
  short getDesiredVelocity(uint8_t servo_id);
  short getCurrentVelocity(uint8_t servo_id);
  short getPreviousVelocity(uint8_t servo_id);
  unsigned short getDesiredTime(uint8_t servo_id);
  unsigned short getRunningTime(uint8_t servo_id);
  unsigned short getWorkingTime(uint8_t servo_id);
  short getDesiredTorque(uint8_t servo_id);
  unsigned long getSystemClock(uint8_t servo_id);
  unsigned short getSamplingTime(uint8_t servo_id);
  short getMCUTemp(uint8_t servo_id);
  short getMotorTemp(uint8_t servo_id);
  short getCurrent(uint8_t servo_id);
  unsigned short getInputVoltage(uint8_t servo_id);
  unsigned short getPWMCount(uint8_t servo_id);
  unsigned short getPWMFrequency(uint8_t servo_id);
  unsigned short getCurrentEncoderValue(uint8_t servo_id);
  long getTotalEncoderCount(uint8_t servo_id);
  uint8_t getHallICState(uint8_t servo_id);

  // control parameter related functions
  bool setPIDGainPresetNo(uint8_t servo_id, uint8_t no);
  bool setPGain0(uint8_t servo_id, unsigned long p_gain);
  bool setIGain0(uint8_t servo_id, unsigned long i_gain);
  bool setDGain0(uint8_t servo_id, unsigned long d_gain);
  bool setStaticFriction0(uint8_t servo_id, unsigned short static_friction);
  bool setDynamicFriction0(uint8_t servo_id, unsigned short dynamic_friction);
  bool setPGain1(uint8_t servo_id, unsigned long p_gain);
  bool setIGain1(uint8_t servo_id, unsigned long i_gain);
  bool setDGain1(uint8_t servo_id, unsigned long d_gain);
  bool setStaticFriction1(uint8_t servo_id, unsigned short static_friction);
  bool setDynamicFriction1(uint8_t servo_id, unsigned short dynamic_friction);
  bool setPGain2(uint8_t servo_id, unsigned long p_gain);
  bool setIGain2(uint8_t servo_id, unsigned long i_gain);
  bool setDGain2(uint8_t servo_id, unsigned long d_gain);
  bool setStaticFriction2(uint8_t servo_id, unsigned short static_friction);
  bool setDynamicFriction2(uint8_t servo_id, unsigned short dynamic_friction);
  uint8_t getPIDGainPresetNo(uint8_t servo_id);
  unsigned long getPGain0(uint8_t servo_id);
  unsigned long getIGain0(uint8_t servo_id);
  unsigned long getDGain0(uint8_t servo_id);
  unsigned short getStaticFriction0(uint8_t servo_id);
  unsigned short getDynamicFriction0(uint8_t servo_id);
  unsigned long getPGain1(uint8_t servo_id);
  unsigned long getIGain1(uint8_t servo_id);
  unsigned long getDGain1(uint8_t servo_id);
  unsigned short getStaticFriction1(uint8_t servo_id);
  unsigned short getDynamicFriction1(uint8_t servo_id);
  unsigned long getPGain2(uint8_t servo_id);
  unsigned long getIGain2(uint8_t servo_id);
  unsigned long getDGain2(uint8_t servo_id);
  unsigned short getStaticFriction2(uint8_t servo_id);
  unsigned short getDynamicFriction2(uint8_t servo_id);

  // version information functions
  char getModelVoltageClass(uint8_t servo_id);
  uint8_t getModelVersion(uint8_t servo_id);
  uint8_t getModelTorqueNumber(uint8_t servo_id);
  uint8_t getModelCaseNumber(uint8_t servo_id);
  char getModelMotorType(uint8_t servo_id);
  char getModelDeviceType(uint8_t servo_id);
  uint8_t getFirmwareBuildNumber(uint8_t servo_id);
  uint8_t getFirmwareRevisionNumber(uint8_t servo_id);
  uint8_t getFirmwareMinorVersion(uint8_t servo_id);
  uint8_t getFirmwareMajorVersion(uint8_t servo_id);
};  // class B3MInterface
}  // namespace b3m_driver
#endif  // B3M_INTERFACE_HPP_
