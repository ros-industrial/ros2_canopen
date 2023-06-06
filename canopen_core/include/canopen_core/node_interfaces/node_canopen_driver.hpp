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

#ifndef NODE_CANOPEN_DRIVER_HPP_
#define NODE_CANOPEN_DRIVER_HPP_

#include <yaml-cpp/yaml.h>
#include <atomic>
#include <lely/coapp/driver.hpp>
#include <map>
#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_interfaces/node_clock_interface.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>
#include <rclcpp/node_interfaces/node_time_source_interface.hpp>
#include <rclcpp/node_interfaces/node_timers_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_waitables_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <type_traits>
#include <vector>
#include "canopen_core/driver_error.hpp"
#include "canopen_core/node_interfaces/node_canopen_driver_interface.hpp"
#include "canopen_core/visibility_control.h"
#include "canopen_interfaces/srv/co_node.hpp"

using namespace rclcpp;
namespace ros2_canopen
{
namespace node_interfaces
{
/**
 * @brief Node Canopen Driver
 *
 * This class implements the NodeCanopenDriverInterface. It provides
 * core functionality and logic for CanopenDriver, indepentently of the
 * ROS node type. Currently rclcpp::Node and rclcpp_lifecycle::LifecycleNode
 * and derived classes are supported. Other node types will lead to compile
 * time error.
 *
 * @tparam NODETYPE
 */
template <class NODETYPE>
class NodeCanopenDriver : public NodeCanopenDriverInterface
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  NODETYPE * node_;

  std::shared_ptr<lely::ev::Executor> exec_;
  std::shared_ptr<lely::canopen::AsyncMaster> master_;
  std::shared_ptr<lely::canopen::BasicDriver> driver_;

  std::chrono::milliseconds non_transmit_timeout_;
  YAML::Node config_;
  uint8_t node_id_;
  std::string container_name_;
  std::string eds_;
  std::string bin_;

  rclcpp::CallbackGroup::SharedPtr client_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;

  std::atomic<bool> master_set_;
  std::atomic<bool> initialised_;
  std::atomic<bool> configured_;
  std::atomic<bool> activated_;

public:
  NodeCanopenDriver(NODETYPE * node)
  : master_set_(false), initialised_(false), configured_(false), activated_(false)
  {
    node_ = node;
  }

  /**
   * @brief Set Master
   *
   * Sets the Lely Canopen Master Objects needed by the driver
   * to add itself to the master.
   *
   */
  virtual void set_master(
    std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master)
  {
    RCLCPP_DEBUG(node_->get_logger(), "set_master_start");
    if (!configured_.load())
    {
      throw DriverException("Set Master: driver is not configured");
    }
    if (activated_.load())
    {
      throw DriverException("Set Master: driver is not activated");
    }
    this->exec_ = exec;
    this->master_ = master;
    this->master_set_.store(true);
    RCLCPP_DEBUG(node_->get_logger(), "set_master_end");
  }

  /**
   * @brief Initialise the driver
   *
   * In this function the ROS interface should be created and
   * potentially necessary callback groups.
   *
   */
  void init()
  {
    RCLCPP_DEBUG(node_->get_logger(), "init_start");
    if (configured_.load())
    {
      throw DriverException("Init: Driver is already configured");
    }
    if (activated_.load())
    {
      throw DriverException("Init: Driver is already activated");
    }
    client_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    node_->declare_parameter("container_name", "");
    node_->declare_parameter("node_id", 0);
    node_->declare_parameter("non_transmit_timeout", 100);
    node_->declare_parameter("config", "");
    this->init(true);
    this->initialised_.store(true);
    RCLCPP_DEBUG(node_->get_logger(), "init_end");
  }

  /**
   * @brief Initialises the driver
   *
   * This does not do anything, it is an empty function. it
   * should be overridden by derived classes.
   *
   * @todo
   * Potentially make this an abstract function. This is mainly
   * for debugging purposes.
   *
   * @param called_from_base
   */
  virtual void init(bool called_from_base) {}

  /**
   * @brief Configure the driver
   *
   * This function should contain the configuration related things,
   * such as reading parameter data or configuration data from files.
   *
   * This function reads the parameters container_name, non_transmit_timeout,
   * node_id and config. Once done it will call the configure(bool) function
   * that should be over
   *
   */
  void configure()
  {
    RCLCPP_DEBUG(node_->get_logger(), "configure_start");
    if (!initialised_.load())
    {
      throw DriverException("Configure: driver is not initialised");
    }
    if (configured_.load())
    {
      throw DriverException("Configure: driver is already configured");
    }
    if (activated_.load())
    {
      throw DriverException("Configure: driver is already activated");
    }
    int non_transmit_timeout;
    std::string config;
    node_->get_parameter("container_name", container_name_);
    node_->get_parameter("non_transmit_timeout", non_transmit_timeout);
    node_->get_parameter("node_id", this->node_id_);
    node_->get_parameter("config", config);
    this->config_ = YAML::Load(config);
    this->non_transmit_timeout_ = std::chrono::milliseconds(non_transmit_timeout);
    auto path = this->config_["dcf_path"].as<std::string>();
    auto dcf = this->config_["dcf"].as<std::string>();
    auto name = this->node_->get_name();
    eds_ = path + "/" + dcf;
    bin_ = path + "/" + name + ".bin";
    this->configure(true);
    this->configured_.store(true);
    RCLCPP_DEBUG(node_->get_logger(), "configure_end");
  }
  /**
   * @brief Configure the driver
   *
   * This function should be overridden by derived classes.
   *
   * @param called_from_base
   */
  virtual void configure(bool called_from_base) {}

  /**
   * @brief Activate the driver
   *
   * This function should activate the driver, consequently it needs to start all timers and threads
   * necessary for the operation of the driver.
   *
   */
  void activate()
  {
    RCLCPP_DEBUG(node_->get_logger(), "activate_start");
    if (!master_set_.load())
    {
      throw DriverException("Activate: master is not set");
    }
    if (!initialised_.load())
    {
      throw DriverException("Activate: driver is not initialised");
    }
    if (!configured_.load())
    {
      throw DriverException("Activate: driver is not configured");
    }
    if (activated_.load())
    {
      throw DriverException("Activate: driver is already activated");
    }
    this->add_to_master();
    this->activate(true);
    this->activated_.store(true);
    RCLCPP_DEBUG(node_->get_logger(), "activate_end");
  }

  /**
   * @brief Activates the driver.
   *
   * This function should be overridden by derived
   * classes.
   *
   * @param called_from_base
   */
  virtual void activate(bool called_from_base) {}

  /**
   * @brief Deactivate the driver
   *
   * This function should deactivate the driver, consequently it needs to stop all timers and
   * threads that are related to the operation of the diver.
   *
   */
  void deactivate()
  {
    RCLCPP_DEBUG(node_->get_logger(), "deactivate_start");
    if (!master_set_.load())
    {
      throw DriverException("Activate: master is not set");
    }
    if (!initialised_.load())
    {
      throw DriverException("Deactivate: driver is not initialised");
    }
    if (!configured_.load())
    {
      throw DriverException("Deactivate: driver is not configured");
    }
    if (!activated_.load())
    {
      throw DriverException("Deactivate: driver is not activated");
    }
    this->activated_.store(false);
    this->remove_from_master();
    this->deactivate(true);
    RCLCPP_DEBUG(node_->get_logger(), "deactivate_end");
  }
  /**
   * @brief Deactivates the driver
   *
   * This function should be overridden by derived classes.
   *
   * @param called_from_base
   */
  virtual void deactivate(bool called_from_base) {}

  /**
   * @brief Cleanup the driver
   *
   * This function needs to clean the internal state of the driver. This means
   * all data should be deleted.
   *
   */
  void cleanup()
  {
    if (!initialised_.load())
    {
      throw DriverException("Cleanup: driver is not initialised");
    }
    if (!configured_.load())
    {
      throw DriverException("Cleanup: driver is not configured");
    }
    if (activated_.load())
    {
      throw DriverException("Cleanup: driver is still activated");
    }
    this->configured_.store(false);
  }

  /**
   * @brief Cleanup the driver
   *
   * This function should be overridden by derived classes.
   *
   * @param called_from_base
   */
  virtual void cleanup(bool called_from_base) {}

  /**
   * @brief Shutdown the driver
   *
   * This function should shutdown the driver.
   *
   */
  void shutdown()
  {
    RCLCPP_DEBUG(this->node_->get_logger(), "Shutting down.");
    if (this->activated_)
    {
      this->deactivate();
    }
    if (this->configured_)
    {
      this->cleanup();
    }
    shutdown(true);
    this->master_set_.store(false);
    this->initialised_.store(false);
    this->configured_.store(false);
    this->activated_.store(false);
  }
  /**
   * @brief Shuts down the driver
   *
   * This function should be overridden by derived classes.
   *
   * @param called_from_base
   */
  virtual void shutdown(bool called_from_base) {}

  virtual void demand_set_master();

protected:
  /**
   * @brief Add the driver to master
   *
   */
  virtual void add_to_master() { throw DriverException("Add to master not implemented."); }

  /**
   * @brief Remove the driver from master
   *
   */
  virtual void remove_from_master()
  {
    throw DriverException("Remove from master not implemented.");
  }
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
