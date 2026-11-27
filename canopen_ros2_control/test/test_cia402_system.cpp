// Copyright (c) 2026, ROS-Industrial
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

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
// exported by the CanopenSystem base class for every node
const std::vector<std::string> kCanopenStates = {
  "rpdo/index",
  "rpdo/subindex",
  "rpdo/data",
  "nmt/state",
  "emcy/error_code",
  "emcy/error_register",
  "emcy/manufacturer_error_code1",
  "emcy/manufacturer_error_code2",
  "emcy/manufacturer_error_code3",
  "emcy/manufacturer_error_code4",
  "emcy/manufacturer_error_code5"};
const std::vector<std::string> kCanopenCommands = {"tpdo/index", "tpdo/subindex", "tpdo/data",
                                                   "tpdo/ons",   "nmt/reset",     "nmt/reset_fbk",
                                                   "nmt/start",  "nmt/start_fbk"};

// added by Cia402System for every motor
const std::vector<std::string> kMotorStates = {"position", "velocity", "effort"};
const std::vector<std::string> kMotorCommands = {
  "position",
  "velocity",
  "effort",
  "init_cmd",
  "init_fbk",
  "halt_cmd",
  "halt_fbk",
  "recover_cmd",
  "recover_fbk",
  "position_mode_cmd",
  "position_mode_fbk",
  "velocity_mode_cmd",
  "velocity_mode_fbk",
  "cyclic_velocity_mode_cmd",
  "cyclic_velocity_mode_fbk",
  "cyclic_position_mode_cmd",
  "cyclic_position_mode_fbk",
  "interpolated_position_mode_cmd",
  "interpolated_position_mode_fbk"};

void append(
  std::vector<std::string> & names, const std::string & joint,
  const std::vector<std::string> & interfaces)
{
  for (const auto & interface : interfaces)
  {
    names.push_back(joint + "/" + interface);
  }
}
}  // namespace

// Every CANopen and motor interface is exported exactly once, whether or not the URDF declares it.
TEST(TestCia402System, exports_motor_interfaces_once_per_node)
{
  // joint1 declares position like a forward position controller setup, joint2 declares nothing
  const std::string system = R"(
    <ros2_control name="Cia402System" type="system">
      <hardware>
        <plugin>canopen_ros2_control/Cia402System</plugin>
      </hardware>
      <joint name="joint1">
        <param name="node_id">2</param>
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      <joint name="joint2">
        <param name="node_id">3</param>
      </joint>
    </ros2_control>
  )";
  hardware_interface::ResourceManager rm(
    ros2_control_test_assets::urdf_head + system + ros2_control_test_assets::urdf_tail,
    std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test_cia402_system"));

  std::vector<std::string> states;
  std::vector<std::string> commands;
  for (const std::string joint : {"joint1", "joint2"})
  {
    append(states, joint, kCanopenStates);
    append(states, joint, kMotorStates);
    append(commands, joint, kCanopenCommands);
    append(commands, joint, kMotorCommands);
  }
  EXPECT_THAT(rm.state_interface_keys(), ::testing::UnorderedElementsAreArray(states));
  EXPECT_THAT(rm.command_interface_keys(), ::testing::UnorderedElementsAreArray(commands));
}
