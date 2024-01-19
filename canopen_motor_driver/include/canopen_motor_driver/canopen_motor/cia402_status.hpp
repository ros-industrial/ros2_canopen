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

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_STATUS_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_STATUS_HPP_
#include <cinttypes>

namespace ros2_canopen
{

#define SW_NOT_READY_TO_SWITCH_ON_MASK 0x4F
#define SW_SWITCH_ON_DISABLED_MASK 0x4F
#define SW_READY_TO_SWITCH_ON_MASK 0x6F
#define SW_SWITCHED_ON_MASK 0x7F
#define SW_OPERATION_ENABLED_MASK 0x7F
#define SW_QUICK_STOP_ACTIVE_MASK 0x6F
#define SW_FAULT_REACTION_ACTIVE_MASK 0x4F
#define SW_FAULT_MASK 0x4F
#define SW_TARGET_REACHED_MASK 0x0400
#define SW_INTERNAL_LIMIT_ACTIVE_MASK 0x0200
#define SW_VOLTAGE_ENABLED_MASK 0x0010
#define SW_OPERATION_MODE1_MASK 0x1000
#define SW_OPERATION_MODE2_MASK 0x2000
#define SW_OPERATION_MODE3_MASK 0x4000

enum class Status : uint8_t
{
  NOT_READY_TO_SWITCH_ON = 0x00,
  SWITCH_ON_DISABLED = 0x40,
  READY_TO_SWITCH_ON = 0x21,
  SWITCHED_ON = 0x33,
  OPERATION_ENABLED = 0x37,
  QUICK_STOP_ACTIVE = 0x07,
  FAULT_REACTION_ACTIVE = 0x0F,
  FAULT = 0x08,
  UNKNOWN = 0xFF
};

Status get_status(uint16_t status_word) noexcept;

bool get_status_bit(uint16_t status_word, uint16_t mask) noexcept;

Status GetNextStatus(Status current, Status goal) noexcept;
}  // namespace ros2_canopen
#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_Status_HPP_
