// Copyright (c) 2023, Fraunhofer IPA
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

#ifndef CANOPEN_ROS2_CONTROL__HELPER_HPP_
#define CANOPEN_ROS2_CONTROL__HELPER_HPP_

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"

namespace canopen_ros2_control
{
/**
 * @brief Struct for storing the data necessary for a triggering command.
 *
 * @details
 * A trigger command from a controller will write to the command double.
 * The result of the trigger operation will be written to the response double.
 * A result of 1.0 means the operation was successful.
 * A result of 0.0 means the operation failed.
 * Only a trigger command of 1.0 will be accepted and trigger the operation.
 * The command_available function can be used to check if a command is available,
 * undefined commands (other than 1.0) will be ignored.
 * The set_response function should be used to set the response value.
 * It will then clear the command.
 *
 */
struct TriggerCommand
{
  double command = std::numeric_limits<double>::quiet_NaN();
  double response = std::numeric_limits<double>::quiet_NaN();
  std::function<bool()> trigger_function;

  bool try_trigger()
  {
    if (command == 1.0)
    {
      response = trigger_function() ? 1.0 : 0.0;
      command = std::numeric_limits<double>::quiet_NaN();
      return true;
    }
    command = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
};

/**
 * @brief Internal double exposed as a ros2_control interface.
 *
 * ros2_control owns the interface values, so the hardware copies *value to the state handle in
 * read() and syncs command handles to and from *value around its write() logic. `fetched` holds
 * the command value seen at the start of write(), so only values the hardware changed are written
 * back and concurrent writes from controller service threads are not overwritten.
 */
struct InterfaceBinding
{
  InterfaceBinding(
    const std::string & prefix, const std::string & interface_name, double * value_ptr)
  : description(prefix, make_info(interface_name)), value(value_ptr)
  {
  }

  hardware_interface::InterfaceDescription description;
  double * value;
  double fetched = std::numeric_limits<double>::quiet_NaN();

  bool changed() const
  {
    return !(*value == fetched || (std::isnan(*value) && std::isnan(fetched)));
  }

private:
  static hardware_interface::InterfaceInfo make_info(const std::string & interface_name)
  {
    hardware_interface::InterfaceInfo info;
    info.name = interface_name;
    return info;
  }
};

/**
 * @brief Descriptions of the bindings not declared in the URDF.
 *
 * URDF-declared (listed) interfaces are already exported by ros2_control itself.
 */
inline std::vector<hardware_interface::InterfaceDescription> unlisted_descriptions(
  const std::vector<InterfaceBinding> & bindings,
  const std::unordered_map<std::string, hardware_interface::InterfaceDescription> & listed)
{
  std::vector<hardware_interface::InterfaceDescription> descriptions;
  for (const auto & binding : bindings)
  {
    if (listed.find(binding.description.get_name()) == listed.end())
    {
      descriptions.push_back(binding.description);
    }
  }
  return descriptions;
}
}  // namespace canopen_ros2_control
#endif
