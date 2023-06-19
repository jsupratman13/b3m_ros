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
#include "b3m_driver/b3m_map.hpp"

namespace b3m_driver
{
class B3MDriver
{
public:
  /**
   * @brief open b3m servo serial port
   *
   * @param port is the port name. Should be something like "/dev/ttyUSB0"
   * @param baudrate is the baudrate.
   */
  void open(const std::string& port, uint32_t baudrate)
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

  /**
   * @brief close b3m servo serial port
   *
   */
  void close()
  {
    serial_.close();
  }

  /**
   * @brief Save RAM parameters to ROM for multiple servos
   *
   * @param servo_id is the servo id to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @return uint8_t is the error code depending on what type of error response. 0 means no error.
   */
  uint8_t save(uint8_t servo_id, uint8_t option)
  {
    std::vector<uint8_t> servo_ids = { servo_id };
    save(servo_ids, option);

    serial_.flushInput();
    serial_.waitReadable();
    std::vector<uint8_t> recv_data;
    serial_.read(recv_data, 5);
    return recv_data[2];
  }

  /**
   * @brief Save RAM parameters to ROM for multiple servos
   *
   * @param servo_ids is the list of servo ids to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   */
  void save(std::vector<uint8_t> servo_ids, uint8_t option)
  {
    serial_.flushOutput();

    std::vector<uint8_t> send_data;
    send_data.push_back(4 + servo_ids.size());
    send_data.push_back(COMMAND_TYPE_SAVE);
    send_data.push_back(option);
    send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
    send_data.push_back(checkSum(send_data));
    serial_.write(send_data);
  }

  /**
   * @brief Load ROM parameters to RAM for single servo
   *
   * @param servo_id is the servo id to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @return uint8_t is the error code depending on what type of error response. 0 means no error.
   */
  uint8_t load(uint8_t servo_id, uint8_t option)
  {
    std::vector<uint8_t> servo_ids = { servo_id };
    load(servo_ids, option);

    serial_.flushInput();
    serial_.waitReadable();
    std::vector<uint8_t> recv_data;
    serial_.read(recv_data, 5);
    return recv_data[2];
  }

  /**
   * @brief Load ROM parameters to RAM for multiple servos
   *
   * @param servo_ids is the list of servo ids to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   */
  void load(std::vector<uint8_t> servo_ids, uint8_t option)
  {
    serial_.flushOutput();

    std::vector<uint8_t> send_data;
    send_data.push_back(4 + servo_ids.size());
    send_data.push_back(COMMAND_TYPE_LOAD);
    send_data.push_back(option);
    send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
    send_data.push_back(checkSum(send_data));
    serial_.write(send_data);
  }

  /**
   * @brief Reset servo
   *
   * @param servo_id is the servo id to send command to.
   * @param time is when to reset.
   */
  void reset(uint8_t servo_id, uint8_t time)
  {
    std::vector<uint8_t> servo_ids = { servo_id };
    reset(servo_ids, time);
  }

  /**
   * @brief Reset multiple servo
   *
   * @param servo_ids is the list of servo ids to send command to.
   * @param time is when to reset.
   */
  void reset(std::vector<uint8_t> servo_ids, uint8_t time)
  {
    serial_.flushOutput();

    std::vector<uint8_t> send_data;
    send_data.push_back(5 + servo_ids.size());
    send_data.push_back(COMMAND_TYPE_RESET);
    send_data.push_back(CLEAR_ERROR);
    send_data.insert(send_data.end(), servo_ids.begin(), servo_ids.end());
    send_data.push_back(time);
    send_data.push_back(checkSum(send_data));
    serial_.write(send_data);
  }

  /**
   * @brief Write data to single servo
   *
   * @param T is the data type to send. Should be either uint8_t, short, unsigned short, long or unsigned long
   * @param servo_id is the servo id to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @param data is the data to send.
   * @param address is the address to send data to.
   * @return uint8_t is the error code depending on what type of error response. 0 means no error.
   */
  template <typename T>
  uint8_t write(uint8_t servo_id, uint8_t option, T data, uint8_t address)
  {
    std::vector<T> multi_data = { data };
    std::vector<uint8_t> servo_ids = { servo_id };
    write<T>(servo_ids, option, multi_data, address);

    serial_.flushInput();
    serial_.waitReadable();

    std::vector<uint8_t> recv_data;
    serial_.read(recv_data, 5);

    return recv_data[2];
  }

  /**
   * @brief Write data to multiple servos
   *
   * @param T is the data type to send. Should be either uint8_t, short, unsigned short, long or unsigned long
   * @param servo_ids is the list of servo ids to send command to.
   * @param option is the type of error response. Should be either
   * RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS, RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @param data is the list of data to send.
   * @param address is the address to send data to.
   */
  template <typename T>
  void write(std::vector<uint8_t> servo_ids, int8_t option, std::vector<T> data, uint8_t address)
  {
    serial_.flushOutput();

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

    serial_.write(send_data);
  }

  /**
   * @brief Set position for single servo
   *
   * @param servo_id is the servo id to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @param position is the position to move to.
   * @param duration is the duration to move to the given position.
   * @return uint8_t is the error code depending on what type of error response. 0 means no error.
   */
  uint8_t setPosition(uint8_t servo_id, uint8_t option, short position, unsigned short duration)
  {
    std::vector<short> multi_positions = { position };
    std::vector<uint8_t> servo_ids = { servo_id };
    setPosition(servo_ids, option, multi_positions, duration);

    serial_.flushInput();
    serial_.waitReadable();

    std::vector<uint8_t> recv_data;
    serial_.read(recv_data, 7);

    return recv_data[2];
  }

  /**
   * @brief Set position for multiple servos
   *
   * @param servo_ids is the list of servo ids to send command to.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @param positions is the list of positions to move to.
   * @param duration is the duration to move to the given position.
   */
  void setPosition(std::vector<uint8_t> servo_ids, uint8_t option, std::vector<short> positions,
                   unsigned short duration)
  {
    serial_.flushOutput();

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
    send_data.push_back(option);
    send_data.insert(send_data.end(), multi_data_bytes.begin(), multi_data_bytes.end());
    send_data.insert(send_data.end(), duration_bytes.begin(), duration_bytes.end());
    send_data.push_back(checkSum(send_data));
    serial_.write(send_data);
  }

  /**
   * @brief read status
   *
   * @tparam T is the data type to send. Should be either uint8_t, short, unsigned short, long or unsigned long
   * @param servo_id is the servo id to read from.
   * @param option is the type of error response. Should be either RETURN_ERROR_STATUS, RETURN_MOTOR_STATUS,
   * RETURN_UART_STATUS, or RETURN_COMMAND_STATUS or CLEAR_ERROR.
   * @param address is the address to read from.
   * @param length is the expected data length to read.
   * @param data is the data to read into. The read data will be stored in this variable.
   * @return uint8_t is the error code depending on what type of error response. 0 means no error.
   */
  template <typename T>
  uint8_t read(uint8_t servo_id, uint8_t option, uint8_t address, int length, T& data)
  {
    serial_.flushOutput();

    uint8_t data_length = 7;
    std::vector<uint8_t> send_data;
    send_data.push_back(data_length);
    send_data.push_back(COMMAND_TYPE_READ);
    send_data.push_back(option);
    send_data.push_back(servo_id);
    send_data.push_back(address);
    send_data.push_back(length);
    send_data.push_back(checkSum(send_data));
    serial_.write(send_data);

    serial_.flushInput();
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    serial_.waitReadable();

    std::vector<uint8_t> recv_data;
    serial_.read(recv_data, 5 + length);
    std::vector<uint8_t> data_bytes(recv_data.begin() + 4, recv_data.end() - 1);
    data = fromLittleEndianBytes<T>(data_bytes);

    return recv_data[2];
  }

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
#endif  // B3M_DRIVER_HPP_
