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

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_CONTROL_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_CONTROL_HPP_
#include <cinttypes>
#include "cia402_status.hpp"
namespace ros2_canopen
{

#define CW_SHUTDOWN_MASK 0x87
#define CW_SWITCH_ON_MASK 0x8F
#define CW_DISABLE_VOLTAGE_MASK 0x02
#define CW_QUICK_STOP_MASK 0x06
#define CW_DISABLE_OPERATION_MASK 0x8F
#define CW_ENABLE_OPERATION_MASK 0x8F
#define CW_FAULT_RESET_MASK 0x80
#define CW_OPERATION_MODE1_MASK 0x0010
#define CW_OPERATION_MODE2_MASK 0x0020
#define CW_OPERATION_MODE3_MASK 0x0040
#define CW_HALT_MASK 0x0100

enum class Control : uint16_t
{
  SHUTDOWN = 0x06,
  SWITCH_ON = 0x07,
  DISABLE_VOLTAGE = 0x00,
  QUICK_STOP = 0x02,
  ENABLE_OPERATION = 0x0F,
  FAULT_RESET = 0x80,
  UNKNOWN = 0xFF
};

void set_command(uint16_t & control_word, Control command) noexcept;
void set_command_bit(uint16_t & control_word, uint16_t mask, bool value) noexcept;
Control GetNextCommand(Status current, Status goal) noexcept;
}  // namespace ros2_canopen
#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_COMMAND_HPP_
