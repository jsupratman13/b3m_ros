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

#include "b3m_driver/b3m_driver.hpp"

void B3MDriver::open(const std::string& port, uint32_t baudrate)
{
  serial_.setPort(port);
  serial_.setBaudrate(baudrate);
  serial_.setBytesize(serial::eightbits);
  serial_.setParity(serial::parity_none);
  serial_.setStopbits(serial::stopbits_one);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_.setTimeout(timeout);
  serial_.open();
}

void B3MDriver::close()
{
  serial_.close();
}

/**
 * @brief send single command
 *
 * @param command_type is the type of send command. Should be either COMMAND_TYPE_LOAD or COMMAND_TYPE_SAVE
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_id is the servo id to send command to.
 * @return uint8_t is the error code depending on what type of error response. 0 means no error.
 */
uint8_t B3MDriver::send(uint8_t command_type, uint8_t option, uint8_t servo_id)
{
  std::vector<uint8_t> servo_ids = { servo_id };
  send(command_type, option, servo_ids);

  std::vector<uint8_t> recv_data(5);
  serial_.read(recv_data, 5);
  return recv_data[2];
}

/**
 * @brief send multiple command
 *
 * @param command_type is the type of send command. Should be either COMMAND_TYPE_LOAD or COMMAND_TYPE_SAVE
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_ids is the list of servo ids to send command to.
 */
void B3MDriver::send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids)
{
  uint8_t data_length = 4 + servo_ids.size();
  std::vector<uint8_t> send_data;
  send_data.push_back(data_length);
  send_data.push_back(command_type);
  send_data.push_back(option);
  send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
  send_data.push_back(checkSum(send_data));
  serial_.write(send_data);
}

/**
 * @brief Send single command with data
 *
 * @param T is the data type to send. Should be either uint8_t, short, unsigned short, long or unsigned long
 * @param command_type is the send command. Should be COMMAND_TYPE_WRITE.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_id is the servo id to send command to.
 * @param data is the data to send.
 * @param address is the address to send data to.
 * @return uint8_t is the error code depending on what type of error response. 0 means no error.
 */
template <typename T>
uint8_t B3MDriver::send(uint8_t command_type, uint8_t option, uint8_t servo_id, T data, uint8_t address)
{
  std::vector<T> multi_data = { data };
  std::vector<uint8_t> servo_ids = { servo_id };
  send(command_type, option, servo_ids, multi_data, address);

  std::vector<uint8_t> recv_data(5);
  serial_.read(recv_data, 5);
  return recv_data[2];
}

/**
 * @brief Send multiple command with data
 *
 * @tparam T is the data type to send. Should be either uint8_t, short, unsigned short, long or unsigned long
 * @param command_type is the send command. Should be COMMAND_TYPE_WRITE.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_ids is the list of servo ids to send command to.
 * @param data is the list of data to send.
 * @param address is the address to send data to.
 */
template <typename T>
void B3MDriver::send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, std::vector<T> data,
                     uint8_t address)
{
  std::vector<uint8_t> multi_data_bytes;
  for (std::size_t index; index < servo_ids.size(); index++)
  {
    multi_data_bytes.push_back(servo_ids[index]);
    std::vector<uint8_t> data_bytes = toLittleEndianBytes(data[index]);
    multi_data_bytes.insert(multi_data_bytes.end(), data_bytes.begin(), data_bytes.end());
  }

  uint8_t data_length = 6 + multi_data_bytes.size();
  std::vector<uint8_t> send_data(data_length);
  send_data.push_back(data_length);
  send_data.push_back(command_type);
  send_data.push_back(option);
  send_data.insert(send_data.end(), multi_data_bytes.begin(), multi_data_bytes.end());
  send_data.push_back(address);
  send_data.push_back(servo_ids.size());
  send_data.push_back(checkSum(send_data));
  serial_.write(send_data);
}

/**
 * @brief send single command for reset
 *
 * @param command_type is the type of send command. Should be COMMAND_TYPE_RESET.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_id is the servo id to send command to.
 * @param time is the time to reset
 * @return uint8_t is the error code depending on what type of error response. 0 means no error.
 */
uint8_t B3MDriver::send(uint8_t command_type, uint8_t option, uint8_t servo_id, uint8_t time)
{
  std::vector<uint8_t> servo_ids = { servo_id };
  send(command_type, option, servo_ids, time);

  std::vector<uint8_t> recv_data(5);
  serial_.read(recv_data, 5);
  return recv_data[2];
}

/**
 * @brief send multiple command for reset
 *
 * @param command_type is the type of send command. Should be COMMAND_TYPE_RESET.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_ids is the list of servo ids to send command to.
 * @param time is the time to reset
 */
void B3MDriver::send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, uint8_t time)
{
  uint8_t data_length = 5 + servo_ids.size();
  std::vector<uint8_t> send_data(data_length);
  send_data.push_back(data_length);
  send_data.push_back(command_type);
  send_data.push_back(option);
  send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
  send_data.push_back(time);
  send_data.push_back(checkSum(send_data));
  serial_.write(send_data);
}

/**
 * @brief send single command for position
 *
 * @param command_type is the type of send command. Should be COMMAND_TYPE_POSITION.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_id is the servo id to send command to.
 * @param position is the position to move to.
 * @param time is the time to move to position.
 * @return uint8_t is the error code depending on what type of error response. 0 means no error.
 */
uint8_t B3MDriver::send(uint8_t command_type, uint8_t option, uint8_t servo_id, short position, unsigned short time)
{
  std::vector<short> multi_positions = { position };
  std::vector<uint8_t> servo_ids = { servo_id };
  send(command_type, option, servo_ids, multi_positions, time);

  std::vector<uint8_t> recv_data(5);
  serial_.read(recv_data, 5);
  return recv_data[2];
}

/**
 * @brief send multiple command for position
 *
 * @param command_type is the type of send command. Should be COMMAND_TYPE_POSITION.
 * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
 * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS
 * @param servo_ids is the list of servo ids to send command to.
 * @param positions is the list of positions to move to.
 * @param time is the time to move to position.
 */
void B3MDriver::send(uint8_t command_type, uint8_t option, std::vector<uint8_t> servo_ids, std::vector<short> positions,
                     unsigned short time)
{
  std::vector<uint8_t> multi_data_bytes;
  for (std::size_t index; index < servo_ids.size(); index++)
  {
    multi_data_bytes.push_back(servo_ids[index]);
    std::vector<uint8_t> data_bytes = toLittleEndianBytes(positions[index]);
    multi_data_bytes.insert(multi_data_bytes.end(), data_bytes.begin(), data_bytes.end());
  }
  std::vector<uint8_t> time_bytes = toLittleEndianBytes(time);

  uint8_t data_length = 6 + multi_data_bytes.size() + time_bytes.size();
  std::vector<uint8_t> send_data(data_length);
  send_data.push_back(data_length);
  send_data.push_back(command_type);
  send_data.push_back(option);
  send_data.insert(send_data.end(), multi_data_bytes.begin(), multi_data_bytes.end());
  send_data.insert(send_data.end(), time_bytes.begin(), time_bytes.end());
  send_data.push_back(checkSum(send_data));
  serial_.write(send_data);
}
