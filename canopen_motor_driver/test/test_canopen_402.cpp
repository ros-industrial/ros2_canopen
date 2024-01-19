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
#include "canopen_motor_driver/canopen_motor/cia402_status.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace ros2_canopen;

TEST(Canopen_402_Status, test_status_not_ready_sw_disabled)
{
  auto status = GetNextStatus(Status::NOT_READY_TO_SWITCH_ON, Status::SWITCH_ON_DISABLED);
  EXPECT_EQ(status, Status::SWITCH_ON_DISABLED);
}

TEST(Canopen_402_Status, test_status_sw_disabled_op_enabled)
{
  auto status = GetNextStatus(Status::SWITCH_ON_DISABLED, Status::OPERATION_ENABLED);
  EXPECT_EQ(status, Status::READY_TO_SWITCH_ON);
}

TEST(Canopen_402_Control, test_control_op_enabled_sw_disabled)
{
  auto control = GetNextCommand(Status::OPERATION_ENABLED, Status::SWITCH_ON_DISABLED);
  EXPECT_EQ(control, Control::DISABLE_VOLTAGE);
}
TEST(Canopen_402_Control, test_control_sw_disabled_rd_switch_on)
{
  auto control = GetNextCommand(Status::SWITCH_ON_DISABLED, Status::READY_TO_SWITCH_ON);
  EXPECT_EQ(control, Control::SHUTDOWN);
}
