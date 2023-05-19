#ifndef NODE_CANOPEN_BASE_DRIVER_IMPL
#define NODE_CANOPEN_BASE_DRIVER_IMPL
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopenBaseDriver<NODETYPE>::NodeCanopenBaseDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenDriver<NODETYPE>(node),
  diagnostic_enabled_(false),
  diagnostic_key_value_(new diagnostic_msgs::msg::KeyValue()),
  diagnostic_status_(new diagnostic_msgs::msg::DiagnosticStatus())
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
    polling_ = this->config_["polling"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Could not polling from config, setting to true.");
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
      RCLCPP_ERROR(this->node_->get_logger(), "Could not read period from config, setting to 10ms");
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
    RCLCPP_ERROR(
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

    diagnostic_status_->name = this->node_->get_name();
    diagnostic_status_->hardware_id = std::to_string(this->node_id_);
    diagnostic_publisher_ = this->node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      std::string(this->node_->get_name()).append("/diagnostic"), 10);
  }
}
template <>
void NodeCanopenBaseDriver<rclcpp::Node>::configure(bool called_from_base)
{
  try
  {
    polling_ = this->config_["polling"].as<bool>();
  }
  catch (...)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Could not polling from config, setting to true.");
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
      RCLCPP_ERROR(this->node_->get_logger(), "Could not read period from config, setting to 10ms");
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
    RCLCPP_ERROR(
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

    diagnostic_status_->name = this->node_->get_name();
    diagnostic_status_->hardware_id = std::to_string(this->node_id_);
    diagnostic_publisher_ = this->node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      std::string(this->node_->get_name()).append("/diagnostic"), 10);
  }
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
    diagnostic_timer_ = this->node_->create_wall_timer(
      std::chrono::milliseconds(diagnostic_period_ms_),
      std::bind(&NodeCanopenBaseDriver<NODETYPE>::diagnostic_timer_callback, this),
      this->timer_cbg_);
  }
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::deactivate(bool called_from_base)
{
  nmt_state_publisher_thread_.join();
  poll_timer_->cancel();
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::cleanup(bool called_from_base)
{
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
        this->bin_);
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
    diagnostic_status_->level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnostic_status_->message = "Device booted.";
    diagnostic_status_->values.push_back(
      diagnostic_key_value_->set__key("Add to master").set__value("OK"));
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
    diagnostic_status_->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnostic_status_->message = "Device removed from the master.";
    diagnostic_status_->values.push_back(
      diagnostic_key_value_->set__key("Remove to master").set__value("OK"));
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
  diagnostic_status_->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  diagnostic_status_->message = "Emergency message received.";
  std::string emcy_msg = "Emergency message received. ";
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
  diagnostic_status_->values.push_back(
    diagnostic_key_value_->set__key("EMCY").set__value(emcy_msg));
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
void NodeCanopenBaseDriver<NODETYPE>::diagnostic_timer_callback()
{
}

#endif
