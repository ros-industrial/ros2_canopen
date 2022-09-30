//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef CANOPEN_402_DRIVER__402_DRIVER_HPP_
#define CANOPEN_402_DRIVER__402_DRIVER_HPP_
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_node.hpp"


namespace ros2_canopen
{
  /**
   * @brief Abstract Class for a CANopen Device Node
   *
   * This class provides the base functionality for creating a
   * CANopen device node. It provides callbacks for nmt and rpdo.
   */
  class Cia402Driver : public ros2_canopen::CanopenDriver
  {
    public:
    Cia402Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
  };
} // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_