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

#include "canopen_motor_driver/canopen_motor/cia402_control.hpp"

using namespace ros2_canopen;

void ros2_canopen::set_command(uint16_t & control_word, Control command) noexcept
{
  switch (command)
  {
    case Control::SHUTDOWN:
      control_word &= ~uint16_t(CW_SHUTDOWN_MASK);
      control_word |= uint16_t(command);
      break;
    case Control::SWITCH_ON:
      control_word &= ~uint16_t(CW_SWITCH_ON_MASK);
      control_word |= uint16_t(command);
      break;
    case Control::DISABLE_VOLTAGE:
      control_word &= ~uint16_t(CW_DISABLE_VOLTAGE_MASK);
      control_word |= uint16_t(command);
      break;
    case Control::QUICK_STOP:
      control_word &= ~uint16_t(CW_QUICK_STOP_MASK);
      control_word |= uint16_t(command);
      break;
    case Control::ENABLE_OPERATION:
      control_word &= ~uint16_t(CW_ENABLE_OPERATION_MASK);
      control_word |= uint16_t(command);
      break;
    case Control::FAULT_RESET:
      control_word &= ~uint16_t(CW_FAULT_RESET_MASK);
      control_word |= uint16_t(command);
      break;
  }
}

void ros2_canopen::set_command_bit(uint16_t & control_word, uint16_t mask, bool value) noexcept
{
  if (value)
  {
    control_word |= mask;
  }
  else
  {
    control_word &= ~mask;
  }
}

Control ros2_canopen::GetNextCommand(Status current, Status goal) noexcept
{
  switch (current)
  {
    case Status::NOT_READY_TO_SWITCH_ON:
      return Control::UNKNOWN;
    case Status::SWITCH_ON_DISABLED:
      switch (goal)
      {
        case Status::READY_TO_SWITCH_ON:
          return Control::SHUTDOWN;
        default:
          return Control::UNKNOWN;
      }
    case Status::READY_TO_SWITCH_ON:
      switch (goal)
      {
        case Status::SWITCHED_ON:
          return Control::SWITCH_ON;
        case Status::SWITCH_ON_DISABLED:
          return Control::DISABLE_VOLTAGE;
        default:
          return Control::UNKNOWN;
      }
    case Status::SWITCHED_ON:
      switch (goal)
      {
        case Status::OPERATION_ENABLED:
          return Control::ENABLE_OPERATION;
        case Status::SWITCH_ON_DISABLED:
          return Control::DISABLE_VOLTAGE;
        case Status::READY_TO_SWITCH_ON:
          return Control::SHUTDOWN;
        default:
          return Control::UNKNOWN;
      }
    case Status::OPERATION_ENABLED:
      switch (goal)
      {
        case Status::SWITCHED_ON:
          return Control::SWITCH_ON;
        case Status::SWITCH_ON_DISABLED:
          return Control::DISABLE_VOLTAGE;
        case Status::READY_TO_SWITCH_ON:
          return Control::SHUTDOWN;
        case Status::QUICK_STOP_ACTIVE:
          return Control::QUICK_STOP;
        default:
          return Control::UNKNOWN;
      }
    case Status::QUICK_STOP_ACTIVE:
      switch (goal)
      {
        case Status::OPERATION_ENABLED:
          return Control::ENABLE_OPERATION;
        case Status::SWITCH_ON_DISABLED:
          return Control::DISABLE_VOLTAGE;
        default:
          return Control::UNKNOWN;
      }
    case Status::FAULT:
      switch (goal)
      {
        case Status::SWITCH_ON_DISABLED:
          return Control::FAULT_RESET;
        default:
          return Control::UNKNOWN;
      }
    default:
      return Control::UNKNOWN;
  }
  return Control::UNKNOWN;
}
