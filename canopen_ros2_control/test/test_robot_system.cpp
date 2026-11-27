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

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

// RobotSystem exports position/velocity/effort states for every motor and one command per
// operation mode configured in the bus config, whether or not the URDF declares them.
TEST(TestRobotSystem, exports_interfaces_from_bus_config_once)
{
  // joint_1 supports position and velocity commands, joint_2 position only
  const auto bus_config = std::filesystem::temp_directory_path() / "test_robot_system_bus.yml";
  std::ofstream(bus_config) << R"(
joint_1:
  node_id: 2
  driver: "ros2_canopen::Cia402Driver"
  position_mode: 1
  velocity_mode: 3
joint_2:
  node_id: 3
  driver: "ros2_canopen::Cia402Driver"
  position_mode: 1
)";

  // states declared in the URDF as in canopen_tests' robot_controller description
  const std::string system = R"(
    <ros2_control name="RobotSystem" type="system">
      <hardware>
        <plugin>canopen_ros2_control/RobotSystem</plugin>
        <param name="bus_config">)" +
                             bus_config.string() + R"(</param>
        <param name="master_config">unused.dcf</param>
        <param name="can_interface_name">vcan0</param>
      </hardware>
      <joint name="joint1">
        <param name="device_name">joint_1</param>
        <state_interface name="position"/>
      </joint>
      <joint name="joint2">
        <param name="device_name">joint_2</param>
        <state_interface name="position"/>
      </joint>
    </ros2_control>
  )";
  hardware_interface::ResourceManager rm(
    ros2_control_test_assets::urdf_head + system + ros2_control_test_assets::urdf_tail,
    std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test_robot_system"));
  std::filesystem::remove(bus_config);

  EXPECT_THAT(
    rm.state_interface_keys(), ::testing::UnorderedElementsAre(
                                 "joint1/position", "joint1/velocity", "joint1/effort",
                                 "joint2/position", "joint2/velocity", "joint2/effort"));
  EXPECT_THAT(
    rm.command_interface_keys(),
    ::testing::UnorderedElementsAre("joint1/position", "joint1/velocity", "joint2/position"));
}
