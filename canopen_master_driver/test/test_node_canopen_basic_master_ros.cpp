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

#include <rclcpp/executors.hpp>
#include <thread>
#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"
#include "gtest/gtest.h"

TEST(NodeCanopenBasicMaster, test_good_sequence_advanced)
{
  rclcpp::init(0, nullptr);
  rclcpp::Node * node = new rclcpp::Node("Node");
  auto interface = new ros2_canopen::node_interfaces::NodeCanopenBasicMaster(node);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());
  std::thread spinner = std::thread([exec] { exec->spin(); });

  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface *>(interface);

  EXPECT_NO_THROW(iface->init());

  rclcpp::Parameter container_name("container_name", "none");
  rclcpp::Parameter master_dcf("master_dcf", "master.dcf");
  rclcpp::Parameter master_bin("master_bin", "");
  rclcpp::Parameter can_interface_name("can_interface_name", "vcan0");
  rclcpp::Parameter node_id("node_id", 1);
  rclcpp::Parameter timeout("non_transmit_timeout", 100);
  rclcpp::Parameter config("config", "");
  node->set_parameter(container_name);
  node->set_parameter(master_dcf);
  node->set_parameter(master_bin);
  node->set_parameter(can_interface_name);
  node->set_parameter(node_id);
  node->set_parameter(timeout);
  node->set_parameter(config);
  EXPECT_NO_THROW(iface->configure());
  rclcpp::shutdown();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (spinner.joinable())
  {
    spinner.join();
  }
}

TEST(NodeCanopenBasicLifecycleMaster, test_good_sequence_advanced)
{
  rclcpp::init(0, nullptr);
  rclcpp_lifecycle::LifecycleNode * node = new rclcpp_lifecycle::LifecycleNode("Node");
  auto interface = new ros2_canopen::node_interfaces::NodeCanopenBasicMaster(node);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node->get_node_base_interface());
  std::thread spinner = std::thread([exec] { exec->spin(); });

  auto iface = static_cast<ros2_canopen::node_interfaces::NodeCanopenMasterInterface *>(interface);

  EXPECT_NO_THROW(iface->init());

  rclcpp::Parameter container_name("container_name", "none");
  rclcpp::Parameter master_dcf("master_dcf", "master.dcf");
  rclcpp::Parameter master_bin("master_bin", "");
  rclcpp::Parameter can_interface_name("can_interface_name", "vcan0");
  rclcpp::Parameter node_id("node_id", 1);
  rclcpp::Parameter timeout("non_transmit_timeout", 100);
  rclcpp::Parameter config("config", "");
  node->set_parameter(container_name);
  node->set_parameter(master_dcf);
  node->set_parameter(master_bin);
  node->set_parameter(can_interface_name);
  node->set_parameter(node_id);
  node->set_parameter(timeout);
  node->set_parameter(config);
  EXPECT_NO_THROW(iface->configure());
  rclcpp::shutdown();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (spinner.joinable())
  {
    spinner.join();
  }
}
