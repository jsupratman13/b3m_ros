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
#ifndef B3M_ASYNC_DRIVER_HPP_
#define B3M_ASYNC_DRIVER_HPP_

#include <async_comm/serial.h>
#include <vector>
#include <map>
#include <queue>

#include "b3m_driver/b3m_map.hpp"

namespace b3m_driver
{
class B3MAsyncInterface
{
public:
  B3MAsyncInterface();
  ~B3MAsyncInterface();
  bool connect(const std::string& port, uint32_t baudrate);
  void disconnect();
  void reset(std::vector<uint8_t> servo_ids, uint8_t time = 0);
  void setServoMode(std::vector<uint8_t> servo_id, std::vector<uint8_t> mode,
                    uint8_t error_option = RETURN_ERROR_STATUS);
  void setTrajectoryType(std::vector<uint8_t> servo_id, std::vector<uint8_t> type,
                         uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions, unsigned short duration = 0,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredVelocity(std::vector<uint8_t> servo_ids, std::vector<short> velocities,
                          uint8_t error_option = RETURN_ERROR_STATUS);
  void setDesiredTorque(std::vector<uint8_t> servo_ids, std::vector<short> torques,
                        uint8_t error_option = RETURN_ERROR_STATUS);

  void requestCurrentPosition(const std::vector<uint8_t>& servo_ids, uint8_t error_option = RETURN_ERROR_STATUS);
  void requestCurrentVelocity(const std::vector<uint8_t>& servo_ids, uint8_t error_option = RETURN_ERROR_STATUS);
  short getCurrentPosition(uint8_t servo_id);
  short getCurrentVelocity(uint8_t servo_id);

private:
  std::unique_ptr<async_comm::Serial> serial_;
  std::map<uint8_t, std::queue<uint8_t> > servo_requests_;
  std::map<uint8_t, short> servo_positions_;
  std::map<uint8_t, short> servo_velocities_;

  void readRequest(uint8_t servo_id, uint8_t option, uint8_t address, int length);
  void responseCallback(const uint8_t* data, size_t length);

  template <typename T>
  void write(std::vector<uint8_t> servo_ids, int8_t option, std::vector<T> data, uint8_t address);

  template <typename T>
  std::vector<uint8_t> toLittleEndianBytes(T data)
  {
    std::vector<uint8_t> bytes(sizeof(data));
    for (size_t i = 0; i < sizeof(data); i++)
    {
      bytes[i] = (data >> (i * 8)) & 0xFF;
    }
    return bytes;
  }

  template <typename T>
  T fromLittleEndianBytes(std::vector<uint8_t> bytes)
  {
    if (bytes.size() != sizeof(T))
    {
      throw std::invalid_argument("bytes size does not match data size");
    }
    T data = 0;
    for (size_t i = 0; i < sizeof(data); i++)
    {
      data |= static_cast<T>(bytes[i]) << (i * 8);
    }
    return data;
  }

  uint8_t checkSum(const std::vector<uint8_t>& data)
  {
    uint16_t sum = 0;
    for (auto byte : data)
    {
      sum += byte;
    }
    return static_cast<uint8_t>(sum & 0xFF);
  }
};
}  // namespace b3m_driver
#endif  // B3M_ASYNC_DRIVER_HPP_
