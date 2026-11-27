// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const std::vector<std::string> kStateInterfaces = {
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

const std::vector<std::string> kCommandInterfaces = {"tpdo/index", "tpdo/subindex", "tpdo/data",
                                                     "tpdo/ons",   "nmt/reset",     "nmt/reset_fbk",
                                                     "nmt/start",  "nmt/start_fbk"};

std::vector<std::string> prefixed(
  const std::vector<std::string> & joints, const std::vector<std::string> & interfaces)
{
  std::vector<std::string> names;
  for (const auto & joint : joints)
  {
    for (const auto & interface : interfaces)
    {
      names.push_back(joint + "/" + interface);
    }
  }
  return names;
}
}  // namespace

class TestCanopenSystem : public ::testing::Test
{
protected:
  // joint1 declares some interfaces in the URDF, joint2 none, joint3 has no node_id
  const std::string canopen_system_ = R"(
    <ros2_control name="CanopenSystem" type="system">
      <hardware>
        <plugin>canopen_ros2_control/CanopenSystem</plugin>
      </hardware>
      <joint name="joint1">
        <param name="node_id">2</param>
        <state_interface name="nmt/state"/>
        <command_interface name="tpdo/data"/>
      </joint>
      <joint name="joint2">
        <param name="node_id">3</param>
      </joint>
      <joint name="joint3">
        <state_interface name="position"/>
      </joint>
    </ros2_control>
  )";

  std::unique_ptr<hardware_interface::ResourceManager> load()
  {
    return std::make_unique<hardware_interface::ResourceManager>(
      ros2_control_test_assets::urdf_head + canopen_system_ + ros2_control_test_assets::urdf_tail,
      std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test_canopen_system"));
  }
};

// Every CANopen interface is exported exactly once, whether or not the URDF declares it.
TEST_F(TestCanopenSystem, exports_canopen_interfaces_once_per_node)
{
  auto rm = load();

  auto expected_states = prefixed({"joint1", "joint2"}, kStateInterfaces);
  expected_states.push_back("joint3/position");
  EXPECT_THAT(rm->state_interface_keys(), ::testing::UnorderedElementsAreArray(expected_states));
  EXPECT_THAT(
    rm->command_interface_keys(),
    ::testing::UnorderedElementsAreArray(prefixed({"joint1", "joint2"}, kCommandInterfaces)));
}

// Destroying a never-configured system must not crash.
TEST_F(TestCanopenSystem, destroy_without_configure)
{
  auto rm = load();
  EXPECT_NO_THROW(rm.reset());
}
