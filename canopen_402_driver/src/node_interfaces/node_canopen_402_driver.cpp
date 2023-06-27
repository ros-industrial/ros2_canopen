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

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver_impl.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopen402Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>;
