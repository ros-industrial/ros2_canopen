//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
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

#include "canopen_master_driver/lifecycle_master_driver.hpp"

namespace ros2_canopen
{

ros2_canopen::LifecycleMasterDriver::LifecycleMasterDriver(const rclcpp::NodeOptions & node_options)
: LifecycleCanopenMaster(node_options)
{
  node_canopen_master_ =
    std::make_shared<node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>>(
      this);
}

}  // namespace ros2_canopen

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleMasterDriver)
