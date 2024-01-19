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

#ifndef CANOPEN_MOTOR_DRIVER__NODE_INTERFACES__NODE_CANOPEN_MOTOR_DRIVER_HPP_
#define CANOPEN_MOTOR_DRIVER__NODE_INTERFACES__NODE_CANOPEN_MOTOR_DRIVER_HPP_

#include <lely/coapp/fiber_driver.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "canopen_motor_driver/canopen_motor/cia402.hpp"
namespace ros2_canopen
{
namespace node_interfaces
{
class NodeCanopenBaseDriver : public NodeCanopenDriver<rclcpp::Node>
{
public:
  void add_to_master() override;
  void init(bool called_from_base) override;
  void configure(bool called_from_base) override;
  void activate(bool called_from_base) override;
  void deactivate(bool called_from_base) override;
  void cleanup(bool called_from_base) override;
  void shutdown(bool called_from_base) override;

protected:
  std::shared_ptr<lely::canopen::FiberDriver> fiber_driver_;
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // CANOPEN_MOTOR_DRIVER__NODE_INTERFACES__NODE_CANOPEN_MOTOR_DRIVER_HPP_
