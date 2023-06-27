//    Copyright 2023 Christoph Hellmann Santos
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//

#include "canopen_402_driver/lifecycle_cia402_driver.hpp"

using namespace ros2_canopen;

LifecycleCia402Driver::LifecycleCia402Driver(rclcpp::NodeOptions node_options)
: LifecycleCanopenDriver(node_options)
{
  node_canopen_402_driver_ =
    std::make_shared<node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(node_canopen_402_driver_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleCia402Driver)
