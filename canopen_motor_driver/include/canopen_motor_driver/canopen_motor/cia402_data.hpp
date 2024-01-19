// Copyright 2024 ROS-Industrial
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_DATA_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_DATA_HPP_

#include <cinttypes>
#include <mutex>
#include "cia402_control.hpp"
#include "cia402_status.hpp"

namespace ros2_canopen
{
class CiA402Data
{
private:
  std::mutex mtx;
  // Raw data
  uint32_t in_supported_modes_;
  uint16_t in_status_word_;
  int8_t in_mode_of_operation_;

  uint16_t out_control_word_;
  uint16_t out_mode_of_operation_;

  // Processed data
  Status status_ = Status::UNKNOWN;
  Control command_ = Control::DISABLE_VOLTAGE;
  bool target_reached_ = true;
  bool internal_limit_active_ = false;
  bool voltage_enabled_ = false;

public:
  // Generate comments for all the getters and setters
  void set_supported_modes(uint32_t supported_modes)
  {
    std::lock_guard<std::mutex> lock(mtx);
    in_supported_modes_ = supported_modes;
  }

  uint32_t get_supported_modes()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return in_supported_modes_;
  }

  void set_status_word(uint16_t status_word)
  {
    std::lock_guard<std::mutex> lock(mtx);
    in_status_word_ = status_word;
  }

  uint16_t get_status_word()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return in_status_word_;
  }

  bool get_status_word_bit(uint16_t mask)
  {
    std::lock_guard<std::mutex> lock(mtx);
    return ros2_canopen::get_status_bit(in_status_word_, mask);
  }

  void set_mode_of_operation_display(int8_t mode_of_operation_display)
  {
    std::lock_guard<std::mutex> lock(mtx);
    in_mode_of_operation_ = mode_of_operation_display;
  }

  int8_t get_mode_of_operation_display()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return in_mode_of_operation_;
  }

  void set_control_word(uint16_t control_word)
  {
    std::lock_guard<std::mutex> lock(mtx);
    out_control_word_ = control_word;
  }

  void set_control_word_bit(uint16_t mask, bool value)
  {
    std::lock_guard<std::mutex> lock(mtx);
    ros2_canopen::set_command_bit(out_control_word_, mask, value);
  }

  uint16_t get_control_word()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return out_control_word_;
  }

  void set_mode_of_operation(uint16_t mode_of_operation)
  {
    std::lock_guard<std::mutex> lock(mtx);
    out_mode_of_operation_ = mode_of_operation;
  }

  uint16_t get_mode_of_operation()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return out_mode_of_operation_;
  }

  void set_status(Status status)
  {
    std::lock_guard<std::mutex> lock(mtx);
    status_ = status;
  }

  Status get_status()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return status_;
  }

  void set_command(Control command)
  {
    std::lock_guard<std::mutex> lock(mtx);
    command_ = command;
  }

  Control get_command()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return command_;
  }

  void set_target_reached(bool target_reached)
  {
    std::lock_guard<std::mutex> lock(mtx);
    target_reached_ = target_reached;
  }

  bool get_target_reached()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return target_reached_;
  }

  void set_internal_limit_active(bool internal_limit_active)
  {
    std::lock_guard<std::mutex> lock(mtx);
    internal_limit_active_ = internal_limit_active;
  }

  bool get_internal_limit_active()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return internal_limit_active_;
  }

  void set_voltage_enabled(bool voltage_enabled)
  {
    std::lock_guard<std::mutex> lock(mtx);
    voltage_enabled_ = voltage_enabled;
  }

  bool get_voltage_enabled()
  {
    std::lock_guard<std::mutex> lock(mtx);
    return voltage_enabled_;
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_DATA_HPP_
