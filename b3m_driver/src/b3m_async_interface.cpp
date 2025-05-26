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
#include <iostream>
#include "b3m_driver/b3m_async_interface.hpp"

namespace b3m_driver
{

B3MAsyncInterface::B3MAsyncInterface()
{
}

B3MAsyncInterface::~B3MAsyncInterface()
{
  disconnect();
}

bool B3MAsyncInterface::connect(const std::string& port, uint32_t baudrate)
{
  serial_ = std::make_unique<async_comm::Serial>(port, baudrate);
  if (!serial_->init())
  {
    std::cout << "Failed to open serial port: " << port << std::endl;
    return false;
  }
  serial_->register_receive_callback([this](const uint8_t* data, size_t length) { responseCallback(data, length); });
  return true;
}

void B3MAsyncInterface::disconnect()
{
  if (serial_)
  {
    serial_->close();
    serial_.reset();
  }
}

void B3MAsyncInterface::reset(std::vector<uint8_t> servo_ids, uint8_t time)
{
  std::vector<uint8_t> send_data;
  send_data.push_back(5 + servo_ids.size());
  send_data.push_back(COMMAND_TYPE_RESET);
  send_data.push_back(CLEAR_ERROR);
  send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
  send_data.push_back(time);
  send_data.push_back(checkSum(send_data));
  serial_->send_bytes(send_data.data(), send_data.size());
}

void B3MAsyncInterface::setServoMode(std::vector<uint8_t> servo_id, std::vector<uint8_t> mode, uint8_t error_option)
{
  write(std::move(servo_id), error_option, std::move(mode), SERVO_SERVO_MODE);
}

void B3MAsyncInterface::setTrajectoryType(std::vector<uint8_t> servo_id, std::vector<uint8_t> type,
                                          uint8_t error_option)
{
  write(std::move(servo_id), error_option, std::move(type), SERVO_RUN_MODE);
}

void B3MAsyncInterface::setDesiredPosition(std::vector<uint8_t> servo_ids, std::vector<short> positions,
                                           unsigned short duration, uint8_t error_option)
{
  std::vector<uint8_t> multi_data_bytes;
  for (std::size_t index = 0; index < servo_ids.size(); index++)
  {
    multi_data_bytes.push_back(servo_ids[index]);
    std::vector<uint8_t> data_bytes = toLittleEndianBytes(positions[index]);
    multi_data_bytes.insert(multi_data_bytes.end(), data_bytes.begin(), data_bytes.end());
  }
  std::vector<uint8_t> duration_bytes = toLittleEndianBytes(duration);

  std::vector<uint8_t> send_data;
  send_data.push_back(4 + multi_data_bytes.size() + duration_bytes.size());
  send_data.push_back(COMMAND_TYPE_POSITION);
  send_data.push_back(error_option);
  send_data.insert(send_data.end(), multi_data_bytes.begin(), multi_data_bytes.end());
  send_data.insert(send_data.end(), duration_bytes.begin(), duration_bytes.end());
  send_data.push_back(checkSum(send_data));

  serial_->send_bytes(send_data.data(), send_data.size());
}

void B3MAsyncInterface::requestCurrentPosition(const std::vector<uint8_t>& servo_ids, uint8_t error_option)
{
  for (const auto& servo_id : servo_ids)
  {
    readRequest(servo_id, error_option, SERVO_CURRENT_POSITION, 2);
  }
}

void B3MAsyncInterface::requestCurrentVelocity(const std::vector<uint8_t>& servo_ids, uint8_t error_option)
{
  for (const auto& servo_id : servo_ids)
  {
    readRequest(servo_id, error_option, SERVO_CURRENT_VELOCITY, 2);
  }
}

short B3MAsyncInterface::getCurrentPosition(uint8_t servo_id)
{
  if (servo_positions_.find(servo_id) != servo_positions_.end())
  {
    return servo_positions_[servo_id];
  }
  return 0;
}

short B3MAsyncInterface::getCurrentVelocity(uint8_t servo_id)
{
  if (servo_velocities_.find(servo_id) != servo_velocities_.end())
  {
    return servo_velocities_[servo_id];
  }
  return 0;
}

template <typename T>
void B3MAsyncInterface::write(std::vector<uint8_t> servo_ids, int8_t option, std::vector<T> data, uint8_t address)
{
  std::vector<uint8_t> multi_data_bytes;
  for (std::size_t index = 0; index < servo_ids.size(); index++)
  {
    multi_data_bytes.push_back(servo_ids[index]);
    std::vector<uint8_t> data_bytes = toLittleEndianBytes(data[index]);
    multi_data_bytes.insert(multi_data_bytes.end(), data_bytes.begin(), data_bytes.end());
  }

  std::vector<uint8_t> send_data;
  send_data.push_back(6 + multi_data_bytes.size());
  send_data.push_back(COMMAND_TYPE_WRITE);
  send_data.push_back(option);
  send_data.insert(send_data.end(), multi_data_bytes.begin(), multi_data_bytes.end());
  send_data.push_back(address);
  send_data.push_back(servo_ids.size());
  send_data.push_back(checkSum(send_data));

  serial_->send_bytes(send_data.data(), send_data.size());
}

void B3MAsyncInterface::readRequest(uint8_t servo_id, uint8_t option, uint8_t address, int length)
{
  uint8_t data_length = 7;
  std::vector<uint8_t> send_data;
  send_data.push_back(data_length);
  send_data.push_back(COMMAND_TYPE_READ);
  send_data.push_back(option);
  send_data.push_back(servo_id);
  send_data.push_back(address);
  send_data.push_back(length);
  send_data.push_back(checkSum(send_data));
  servo_requests_[servo_id].push(address);
  serial_->send_bytes(send_data.data(), data_length);
}

void B3MAsyncInterface::responseCallback(const uint8_t* data, size_t length)
{
  if (debug_mode_)
  {
    std::cout << "Received multiple packets: ";
    for (size_t j = 0; j < length; ++j)
    {
      std::cout << std::hex << static_cast<int>(data[j]) << " ";
    }
    std::cout << std::endl;
  }
  for (size_t i = 0; i < length;)
  {
    auto command_size = static_cast<size_t>(data[i]);
    auto packet_end_index = command_size + i;
    if (packet_end_index > length)
    {
      if (debug_mode_)
        std::cerr << "Expected packet index " << packet_end_index << " exceeds given packet size: " << length
                  << std::endl;
      return;
    }
    std::vector<uint8_t> buffer(data + i, data + packet_end_index);
    i = packet_end_index;
    if (!validateReadSum(buffer))
    {
      if (debug_mode_)
      {
        std::cerr << "Checksum validation failed for packet: ";
        for (const auto& byte : buffer)
        {
          std::cerr << std::hex << static_cast<int>(byte) << " ";
        }
        std::cerr << std::endl;
      }
      return;
    }
    auto command_type = buffer[1];
    // TODO: handle non read command?
    if (command_type != 0x83)
    {
      continue;
    }
    auto servo_id = buffer[3];
    if ((servo_requests_.find(servo_id) == servo_requests_.end()) || servo_requests_[servo_id].empty())
    {
      std::cerr << "No request found for servo ID: " << static_cast<int>(servo_id) << std::endl;
      continue;
    }
    auto address = servo_requests_[servo_id].front();
    servo_requests_[servo_id].pop();
    switch (address)
    {
      case SERVO_CURRENT_POSITION: {
        std::vector<uint8_t> data_bytes(buffer.begin() + 4, buffer.end() - 1);
        short position = fromLittleEndianBytes<short>(data_bytes);
        servo_positions_[servo_id] = position;
        break;
      }
      case SERVO_CURRENT_VELOCITY: {
        std::vector<uint8_t> data_bytes(buffer.begin() + 4, buffer.end() - 1);
        short velocity = fromLittleEndianBytes<short>(data_bytes);
        servo_velocities_[servo_id] = velocity;
        break;
      }
      default:
        std::cerr << "Unknown address: " << static_cast<int>(address) << " for servo ID: " << static_cast<int>(servo_id)
                  << std::endl;
        break;
    }
  }
}
}  // namespace b3m_driver
