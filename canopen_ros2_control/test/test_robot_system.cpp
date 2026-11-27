// Copyright (c) 2026, lkhun9311
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

#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "canopen_ros2_control/robot_system.hpp"

using hardware_interface::CallbackReturn;

namespace
{
std::vector<std::string> error_messages;
rcutils_logging_output_handler_t previous_handler = nullptr;

void record_errors(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t timestamp, const char * format, va_list * args)
{
  if (severity == RCUTILS_LOG_SEVERITY_ERROR)
  {
    va_list args_copy;
    va_copy(args_copy, *args);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args_copy);
    va_end(args_copy);
    error_messages.emplace_back(buffer);
  }
  previous_handler(location, severity, name, timestamp, format, args);
}
}  // namespace

class TestRobotSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    error_messages.clear();
    previous_handler = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(record_errors);

    params_.hardware_info.name = "robot_system_test";
    params_.hardware_info.type = "system";
    params_.hardware_info.hardware_parameters["bus_config"] =
      std::string(TEST_RESOURCES_DIRECTORY) + "/bus_configs/robot_system.yml";
    params_.hardware_info.hardware_parameters["master_config"] = "master.dcf";
    params_.hardware_info.hardware_parameters["can_interface_name"] = "vcan0";
  }

  void TearDown() override { rcutils_logging_set_output_handler(previous_handler); }

  void add_joint(const std::string & name, const std::optional<std::string> & device_name)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    if (device_name.has_value())
    {
      joint.parameters["device_name"] = device_name.value();
    }
    params_.hardware_info.joints.push_back(joint);
  }

  bool logged_error(const std::string & text)
  {
    for (const auto & message : error_messages)
    {
      if (message.find(text) != std::string::npos)
      {
        return true;
      }
    }
    return false;
  }

  hardware_interface::HardwareComponentInterfaceParams params_;
  canopen_ros2_control::RobotSystem robot_system_;
};

TEST_F(TestRobotSystem, on_init_succeeds_with_valid_device)
{
  add_joint("joint_1", "motor_1");
  ASSERT_EQ(robot_system_.on_init(params_), CallbackReturn::SUCCESS);

  std::vector<std::string> state_names;
  for (const auto & interface : robot_system_.export_state_interfaces())
  {
    state_names.push_back(interface.get_name());
  }
  EXPECT_EQ(
    state_names,
    std::vector<std::string>({"joint_1/position", "joint_1/velocity", "joint_1/effort"}));

  std::vector<std::string> command_names;
  for (const auto & interface : robot_system_.export_command_interfaces())
  {
    command_names.push_back(interface.get_name());
  }
  EXPECT_EQ(command_names, std::vector<std::string>({"joint_1/position"}));
}

TEST_F(TestRobotSystem, on_init_fails_without_device_name)
{
  add_joint("joint_1", std::nullopt);
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error("No device_name parameter provided for joint 'joint_1'."));
}

TEST_F(TestRobotSystem, on_init_fails_with_empty_device_name)
{
  add_joint("joint_1", "");
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error("No device_name parameter provided for joint 'joint_1'."));
}

TEST_F(TestRobotSystem, on_init_fails_if_second_joint_has_no_device_name)
{
  add_joint("joint_1", "motor_1");
  add_joint("joint_2", std::nullopt);
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error("No device_name parameter provided for joint 'joint_2'."));
}

TEST_F(TestRobotSystem, on_init_fails_with_unknown_device_name)
{
  add_joint("joint_1", "unknown_device");
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error(
    "Cannot read a valid 'driver' entry for device 'unknown_device' of joint 'joint_1'"));
}

TEST_F(TestRobotSystem, on_init_fails_without_driver_entry)
{
  add_joint("joint_1", "no_driver_device");
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error(
    "Cannot read a valid 'driver' entry for device 'no_driver_device' of joint 'joint_1'"));
}

TEST_F(TestRobotSystem, on_init_fails_with_invalid_driver_entry)
{
  add_joint("joint_1", "sequence_driver_device");
  EXPECT_EQ(robot_system_.on_init(params_), CallbackReturn::ERROR);
  EXPECT_TRUE(logged_error(
    "Cannot read a valid 'driver' entry for device 'sequence_driver_device' of joint 'joint_1'"));
}
