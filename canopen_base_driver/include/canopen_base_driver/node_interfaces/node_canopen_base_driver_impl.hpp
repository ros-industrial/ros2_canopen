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

#ifndef NODE_CANOPEN_BASE_DRIVER_IMPL
#define NODE_CANOPEN_BASE_DRIVER_IMPL
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopenBaseDriver<NODETYPE>::NodeCanopenBaseDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenDriver<NODETYPE>(node),
  diagnostic_enabled_(false),
  diagnostic_collector_(new DiagnosticsCollector())
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::init(bool called_from_base)
{
}

template <>
void NodeCanopenBaseDriver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
  try
  {
    this->non_transmit_timeout_ =
      std::chrono::milliseconds(this->config_["non_transmit_timeout"].as<int>());
  }
  catch (...)
  {
  }
  RCLCPP_INFO_STREAM(
    this->node_->get_logger(),
    "Non transmit timeout" << static_cast<int>(this->non_transmit_timeout_.count()) << "ms");

  try
  {
    polling_ = this->config_["polling"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Could not polling from config, setting to true.");
    polling_ = true;
  }
  if (polling_)
  {
    try
    {
      period_ms_ = this->config_["period"].as<std::uint32_t>();
    }
    catch (...)
    {
      RCLCPP_WARN(this->node_->get_logger(), "Could not read period from config, setting to 10ms");
      period_ms_ = 10;
    }
  }

  // Diagnostic components
  try
  {
    diagnostic_enabled_ = this->config_["diagnostics"]["enable"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_WARN(
      this->node_->get_logger(),
      "Could not read enable diagnostics from config, setting to false.");
    diagnostic_enabled_ = false;
  }
  if (diagnostic_enabled_.load())
  {
    try
    {
      diagnostic_period_ms_ = this->config_["diagnostics"]["period"].as<std::uint32_t>();
    }
    catch (...)
    {
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "Could not read diagnostics period from config, setting to 1000ms");
      diagnostic_period_ms_ = 1000;
    }

    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this->node_);
    diagnostic_updater_->setHardwareID(std::to_string(this->node_id_));
  }

  std::optional<int> sdo_timeout_ms;
  try
  {
    sdo_timeout_ms = std::optional(this->config_["sdo_timeout_ms"].as<int>());
  }
  catch (...)
  {
  }
  sdo_timeout_ms_ = sdo_timeout_ms.value_or(20);
}
template <>
void NodeCanopenBaseDriver<rclcpp::Node>::configure(bool called_from_base)
{
  try
  {
    this->non_transmit_timeout_ =
      std::chrono::milliseconds(this->config_["non_transmit_timeout"].as<int>());
  }
  catch (...)
  {
  }
  RCLCPP_INFO_STREAM(
    this->node_->get_logger(),
    "Non transmit timeout" << static_cast<int>(this->non_transmit_timeout_.count()) << "ms");

  try
  {
    polling_ = this->config_["polling"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Could not polling from config, setting to true.");
    polling_ = true;
  }
  if (polling_)
  {
    try
    {
      period_ms_ = this->config_["period"].as<std::uint32_t>();
    }
    catch (...)
    {
      RCLCPP_WARN(this->node_->get_logger(), "Could not read period from config, setting to 10ms");
      period_ms_ = 10;
    }
  }

  // Diagnostic components
  try
  {
    diagnostic_enabled_ = this->config_["diagnostics"]["enable"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_WARN(
      this->node_->get_logger(),
      "Could not read enable diagnostics from config, setting to false.");
    diagnostic_enabled_ = false;
  }
  if (diagnostic_enabled_.load())
  {
    try
    {
      diagnostic_period_ms_ = this->config_["diagnostics"]["period"].as<std::uint32_t>();
    }
    catch (...)
    {
      RCLCPP_WARN(
        this->node_->get_logger(),
        "Could not read diagnostics period from config, setting to 1000ms");
      diagnostic_period_ms_ = 1000;
    }

    diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(this->node_);
    diagnostic_updater_->setHardwareID(std::to_string(this->node_id_));
  }

  std::optional<int> sdo_timeout_ms;
  try
  {
    sdo_timeout_ms = std::optional(this->config_["sdo_timeout_ms"].as<int>());
  }
  catch (...)
  {
  }
  sdo_timeout_ms_ = sdo_timeout_ms.value_or(20);
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::activate(bool called_from_base)
{
  nmt_state_publisher_thread_ =
    std::thread(std::bind(&NodeCanopenBaseDriver<NODETYPE>::nmt_listener, this));
  emcy_queue_ = this->lely_driver_->get_emcy_queue();
  rpdo_queue_ = this->lely_driver_->get_rpdo_queue();
  if (polling_)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Starting with polling mode.");
    poll_timer_ = this->node_->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      std::bind(&NodeCanopenBaseDriver<NODETYPE>::poll_timer_callback, this), this->timer_cbg_);
  }
  else
  {
    RCLCPP_INFO(this->node_->get_logger(), "Starting with event mode.");
    this->lely_driver_->set_sync_function(
      std::bind(&NodeCanopenBaseDriver<NODETYPE>::poll_timer_callback, this));
  }

  if (diagnostic_enabled_.load())
  {
    RCLCPP_INFO(this->node_->get_logger(), "Starting with diagnostics enabled.");
    diagnostic_updater_->add(
      "diagnostic updater", this, &NodeCanopenBaseDriver<NODETYPE>::diagnostic_callback);
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::deactivate(bool called_from_base)
{
  nmt_state_publisher_thread_.join();
  poll_timer_->cancel();
  emcy_queue_.reset();
  rpdo_queue_.reset();
  if (diagnostic_enabled_.load())
  {
    diagnostic_updater_->removeByName("diagnostic updater");
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::cleanup(bool called_from_base)
{
  NodeCanopenDriver<NODETYPE>::cleanup(called_from_base);
  // this->lely_driver_->unset_sync_function();
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::shutdown(bool called_from_base)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::add_to_master()
{
  RCLCPP_INFO(this->node_->get_logger(), "eds file %s", this->eds_.c_str());
  RCLCPP_INFO(this->node_->get_logger(), "bin file %s", this->bin_.c_str());
  std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyDriverBridge>>> prom;
  prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyDriverBridge>>>();
  std::future<std::shared_ptr<ros2_canopen::LelyDriverBridge>> f = prom->get_future();
  this->exec_->post(
    [this, prom]()
    {
      std::scoped_lock<std::mutex> lock(this->driver_mutex_);
      this->lely_driver_ = std::make_shared<ros2_canopen::LelyDriverBridge>(
        *(this->exec_), *(this->master_), this->node_id_, this->node_->get_name(), this->eds_,
        this->bin_, std::chrono::milliseconds(this->sdo_timeout_ms_));
      this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(this->lely_driver_);
      prom->set_value(lely_driver_);
    });

  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Adding timed out.");
    throw DriverException("add_to_master: adding timed out");
  }
  this->lely_driver_ = f.get();
  this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(this->lely_driver_);
  if (!this->lely_driver_->IsReady())
  {
    RCLCPP_WARN(this->node_->get_logger(), "Wait for device to boot.");
    try
    {
      this->lely_driver_->wait_for_boot();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->node_->get_logger(), e.what());
    }
  }
  RCLCPP_INFO(this->node_->get_logger(), "Driver booted and ready.");

  if (diagnostic_enabled_.load())
  {
    diagnostic_collector_->updateAll(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "Device ready", "DEVICE", "Added to master.");
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::remove_from_master()
{
  std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
  auto f = prom->get_future();
  this->exec_->post(
    [this, prom]()
    {
      this->driver_.reset();
      this->lely_driver_.reset();
      prom->set_value();
    });

  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready)
  {
    throw DriverException("remove_from_master: removing timed out");
  }
  if (diagnostic_enabled_.load())
  {
    diagnostic_collector_->updateAll(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device removed", "DEVICE",
      "Removed from master.");
  }
}
template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::nmt_listener()
{
  while (rclcpp::ok())
  {
    std::future<lely::canopen::NmtState> f;
    {
      std::scoped_lock<std::mutex> lock(this->driver_mutex_);
      f = this->lely_driver_->async_request_nmt();
    }
    while (f.wait_for(this->non_transmit_timeout_) != std::future_status::ready)
    {
      if (!this->activated_.load()) return;
    }
    try
    {
      auto state = f.get();
      if (nmt_state_cb_)
      {
        nmt_state_cb_(state, this->lely_driver_->get_id());
      }
      on_nmt(state);
    }
    catch (const std::future_error & e)
    {
      break;
    }
  }
}
template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::on_rpdo(COData data)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::on_emcy(COEmcy emcy)
{
  diagnostic_collector_->summary(
    diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Emergency message received");
  std::string emcy_msg = "Emergency message: ";
  emcy_msg.append("eec: ");
  emcy_msg.append(std::to_string(emcy.eec));
  emcy_msg.append(" er: ");
  emcy_msg.append(std::to_string(emcy.er));
  emcy_msg.append(" msef: ");
  for (auto & msef : emcy.msef)
  {
    emcy_msg.append(std::to_string(msef));
    emcy_msg.append(" ");
  }
  diagnostic_collector_->add("EMCY", emcy_msg);
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::rdpo_listener()
{
  RCLCPP_INFO(this->node_->get_logger(), "Starting RPDO Listener");
  auto q = lely_driver_->get_rpdo_queue();
  while (rclcpp::ok())
  {
    ros2_canopen::COData rpdo;
    while (!q->wait_and_pop_for(this->non_transmit_timeout_, rpdo))
    {
      if (!this->activated_.load()) return;
    }
    try
    {
      if (rpdo_cb_)
      {
        rpdo_cb_(rpdo, this->lely_driver_->get_id());
      }
      on_rpdo(rpdo);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "RPDO Listener error: " << e.what());
      break;
    }
  }
}
template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::poll_timer_callback()
{
  for (int i = 0; i < 10; i++)
  {
    auto opt = emcy_queue_->try_pop();
    if (!opt.has_value())
    {
      break;
    }
    try
    {
      if (emcy_cb_)
      {
        emcy_cb_(opt.value(), this->lely_driver_->get_id());
      }
      on_emcy(opt.value());
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "EMCY poll error: " << e.what());
      break;
    }
  }
  for (int i = 0; i < 10; i++)
  {
    auto opt = rpdo_queue_->try_pop();
    if (!opt.has_value())
    {
      break;
    }
    try
    {
      if (rpdo_cb_)
      {
        rpdo_cb_(opt.value(), this->lely_driver_->get_id());
      }
      on_rpdo(opt.value());
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "RPDO Poll error: " << e.what());
      break;
    }
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::emcy_listener()
{
  RCLCPP_INFO(this->node_->get_logger(), "Starting EMCY Listener");
  auto q = lely_driver_->get_emcy_queue();
  while (rclcpp::ok())
  {
    ros2_canopen::COEmcy emcy;
    while (!q->wait_and_pop_for(this->non_transmit_timeout_, emcy))
    {
      if (!this->activated_.load()) return;
    }
    try
    {
      if (emcy_cb_)
      {
        emcy_cb_(emcy, this->lely_driver_->get_id());
      }
      on_emcy(emcy);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "EMCY Listener error: " << e.what());
      break;
    }
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::diagnostic_callback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
}

#endif
