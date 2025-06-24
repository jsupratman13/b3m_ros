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
#ifndef B3M_ASYNC_INTERFACE_HPP_
#define B3M_ASYNC_INTERFACE_HPP_

#include <vector>
#include "b3m_driver/b3m_async_driver.hpp"

namespace b3m_driver
{
class B3MAsyncInterface
{
public:
  B3MAsyncInterface();
  ~B3MAsyncInterface();
  void connect(const std::string& port, unsigned int baudrate);
  void disconnect();
  void reset(std::vector<uint8_t> servo_ids, uint8_t time = 0);
  bool setServoMode(uint8_t servo_id, uint8_t mode, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setTrajectoryType(uint8_t servo_id, uint8_t type, uint8_t error_option = RETURN_ERROR_STATUS);
  bool setPIDGainPresetNo(uint8_t servo_id, uint8_t no, uint8_t error_option = RETURN_ERROR_STATUS);

  bool setDesiredPosition(uint8_t servo_id, short position, unsigned short duration = 0,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions, unsigned short duration = 0,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrentPosition(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrentVelocity(uint8_t servo_id, uint8_t error_option = RETURN_ERROR_STATUS);

private:
  B3MAsyncDriver driver_;
};
}  // namespace b3m_driver
#endif  // B3M_ASYNC_INTERFACE_HPP_
