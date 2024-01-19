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

#include "canopen_motor_driver/canopen_motor/cia402_status.hpp"

using namespace ros2_canopen;

Status ros2_canopen::get_status(uint16_t status_word) noexcept
{
  if ((status_word & SW_NOT_READY_TO_SWITCH_ON_MASK) == uint8_t(Status::NOT_READY_TO_SWITCH_ON))
  {
    return Status::NOT_READY_TO_SWITCH_ON;
  }
  else if ((status_word & SW_SWITCH_ON_DISABLED_MASK) == uint8_t(Status::SWITCH_ON_DISABLED))
  {
    return Status::SWITCH_ON_DISABLED;
  }
  else if ((status_word & SW_READY_TO_SWITCH_ON_MASK) == uint8_t(Status::READY_TO_SWITCH_ON))
  {
    return Status::READY_TO_SWITCH_ON;
  }
  else if ((status_word & SW_SWITCHED_ON_MASK) == uint8_t(Status::SWITCHED_ON))
  {
    return Status::SWITCHED_ON;
  }
  else if ((status_word & SW_OPERATION_ENABLED_MASK) == uint8_t(Status::OPERATION_ENABLED))
  {
    return Status::OPERATION_ENABLED;
  }
  else if ((status_word & SW_QUICK_STOP_ACTIVE_MASK) == uint8_t(Status::QUICK_STOP_ACTIVE))
  {
    return Status::QUICK_STOP_ACTIVE;
  }
  else if ((status_word & SW_FAULT_REACTION_ACTIVE_MASK) == uint8_t(Status::FAULT_REACTION_ACTIVE))
  {
    return Status::FAULT_REACTION_ACTIVE;
  }
  else if ((status_word & SW_FAULT_MASK) == uint8_t(Status::FAULT))
  {
    return Status::FAULT;
  }
  else
  {
    return Status::UNKNOWN;
  }
}

bool ros2_canopen::get_status_bit(uint16_t status_word, uint16_t mask) noexcept
{
  return (status_word & mask) == mask;
}

Status ros2_canopen::GetNextStatus(Status current, Status goal) noexcept
{
  if (current == goal)
  {
    return current;
  }
  if (current == Status::UNKNOWN)
  {
    return Status::UNKNOWN;
  }
  if (current == Status::NOT_READY_TO_SWITCH_ON)
  {
    return Status::SWITCH_ON_DISABLED;
  }
  if (current == Status::FAULT_REACTION_ACTIVE)
  {
    return Status::FAULT;
  }
  if (current == Status::SWITCH_ON_DISABLED)
  {
    switch (goal)
    {
      case Status::READY_TO_SWITCH_ON:
      case Status::SWITCHED_ON:
      case Status::OPERATION_ENABLED:
        return Status::READY_TO_SWITCH_ON;
      default:
        return Status::UNKNOWN;
    }
  }
  if (current == Status::READY_TO_SWITCH_ON)
  {
    switch (goal)
    {
      case Status::SWITCHED_ON:
      case Status::OPERATION_ENABLED:
        return Status::SWITCHED_ON;
      case Status::SWITCH_ON_DISABLED:
        return Status::SWITCH_ON_DISABLED;
      default:
        return Status::UNKNOWN;
    }
  }
  if (current == Status::SWITCHED_ON)
  {
    switch (goal)
    {
      case Status::OPERATION_ENABLED:
        return Status::OPERATION_ENABLED;
      case Status::READY_TO_SWITCH_ON:
        return Status::READY_TO_SWITCH_ON;
      case Status::SWITCH_ON_DISABLED:
        return Status::SWITCH_ON_DISABLED;
      default:
        return Status::UNKNOWN;
    }
  }
  if (current == Status::OPERATION_ENABLED)
  {
    switch (goal)
    {
      case Status::SWITCHED_ON:
        return Status::SWITCHED_ON;
      case Status::READY_TO_SWITCH_ON:
        return Status::READY_TO_SWITCH_ON;
      case Status::SWITCH_ON_DISABLED:
        return Status::SWITCH_ON_DISABLED;
      case Status::QUICK_STOP_ACTIVE:
        return Status::QUICK_STOP_ACTIVE;
      default:
        return Status::UNKNOWN;
    }
  }
  if (current == Status::QUICK_STOP_ACTIVE)
  {
    switch (goal)
    {
      case Status::OPERATION_ENABLED:
        return Status::OPERATION_ENABLED;
      case Status::SWITCH_ON_DISABLED:
        return Status::SWITCH_ON_DISABLED;
      default:
        return Status::UNKNOWN;
    }
  }
  if (current == Status::FAULT)
  {
    switch (goal)
    {
      case Status::SWITCH_ON_DISABLED:
      case Status::READY_TO_SWITCH_ON:
      case Status::SWITCHED_ON:
      case Status::OPERATION_ENABLED:
        return Status::SWITCH_ON_DISABLED;
      default:
        return Status::UNKNOWN;
    }
  }
  return Status::UNKNOWN;
}
