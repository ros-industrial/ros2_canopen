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
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <thread>
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "gtest/gtest.h"
using namespace rclcpp_components;

TEST(ComponentLoad, test_load_component_1)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources =
    manager->get_component_resources("canopen_402_driver");

  EXPECT_EQ(2u, resources.size());

  auto factory = manager->create_component_factory(resources[0]);
  auto instance_wrapper =
    factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}

TEST(ComponentLoad, test_load_component_2)
{
  rclcpp::init(0, nullptr);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);

  std::vector<ComponentManager::ComponentResource> resources =
    manager->get_component_resources("canopen_402_driver");

  EXPECT_EQ(2u, resources.size());

  auto factory = manager->create_component_factory(resources[1]);
  auto instance_wrapper =
    factory->create_node_instance(rclcpp::NodeOptions().use_global_arguments(false));

  rclcpp::shutdown();
}
