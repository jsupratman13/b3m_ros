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
#ifndef B3M_DRIVER_HPP_
#define B3M_DRIVER_HPP_

#include <serial/serial.h>
#include <sys/types.h>
#include <cstdint>
#include <string>

namespace b3m_driver
{
class B3MDriver
{
public:
  void open(const std::string& port, uint32_t baudrate);
  void close();

  uint8_t send(uint8_t command_type, uint8_t option, uint8_t servo_id);
  void send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids);
  template <typename T>
  uint8_t send(uint8_t command_type, uint8_t option, uint8_t servo_id, T data, uint8_t address);
  template <typename T>
  void send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, std::vector<T> data, uint8_t address);
  uint8_t send(uint8_t command_type, uint8_t option, uint8_t servo_id, uint8_t time);
  void send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, uint8_t time);
  uint8_t send(uint8_t command_type, uint8_t option, uint8_t servo_id, short position, unsigned short time);
  void send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, std::vector<short> positions,
            unsigned short time);

  template <typename T>
  uint8_t read(uint8_t command_type, uint8_t option, uint8_t servo_id, uint8_t address, uint8_t length, T& data);

private:
  serial::Serial serial_;

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
#endif  // B3M_DRIVER_HPP_
