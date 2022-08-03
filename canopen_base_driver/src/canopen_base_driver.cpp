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
#include <memory>
#include "canopen_base_driver/canopen_base_driver.hpp"

using namespace lely;
using namespace ros2_canopen;
using namespace std::chrono_literals;

void BaseDriver::nmt_listener()
{
  while (rclcpp::ok())
  {
    std::future<lely::canopen::NmtState> f;
    {
      std::scoped_lock<std::mutex> lock (this->driver_mutex_);
      f = driver_->async_request_nmt();
    }
    while (f.wait_for(non_transmit_timeout_) != std::future_status::ready)
    {
      if (!this->activated.load())
        return;
    }
    on_nmt(f.get());
  }
}

void BaseDriver::rdpo_listener()
{
  while (rclcpp::ok())
  {
    std::future<ros2_canopen::COData> f;
    {
      std::scoped_lock<std::mutex> lock (this->driver_mutex_);
      f = driver_->async_request_rpdo();
    }

    while (f.wait_for(non_transmit_timeout_) != std::future_status::ready)
    {
      if (!this->activated.load())
        return;
    }

    on_rpdo(f.get());
  }
}

void BaseDriver::init_from_master(
    std::shared_ptr<ev::Executor> exec,
    std::shared_ptr<canopen::AsyncMaster> master,
    std::shared_ptr<ConfigurationManager> config)
{
  this->exec_ = exec;
  this->master_ = master;
  this->config_ = config;
  this->initialised_ = true;
}

bool BaseDriver::add()
{
  std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyBridge>>> prom;
  prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyBridge>>>();
  std::future<std::shared_ptr<ros2_canopen::LelyBridge>> f = prom->get_future();
  master_->GetExecutor().post([this, prom]()
                              {
                std::scoped_lock<std::mutex> lock (this->driver_mutex_);
                driver_ =
                  std::make_shared<ros2_canopen::LelyBridge>(*exec_, *master_, node_id_);
                driver_->Boot();
                prom->set_value(driver_); });
  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready)
  {
    return false;
  }
  driver_ = f.get();
  return true;
}

bool BaseDriver::remove()
{
  std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
  auto f = prom->get_future();
  exec_->post([this, prom]()
              { 
                                driver_.reset(); 
                                prom->set_value(); });
  auto future_status = f.wait_for(this->non_transmit_timeout_);

  if (future_status != std::future_status::ready)
  {
    return false;
  }
  return true;
}

bool BaseDriver::demand_init_from_master(uint8_t node_id, std::chrono::seconds time_out)
{
  while (!demand_init_from_master_client_->wait_for_service(time_out))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for init_driver service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "init_driver service not available, waiting again...");
  }
  auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
  request->nodeid = node_id;

  auto future_result = demand_init_from_master_client_->async_send_request(request);

  auto future_status = future_result.wait_for(time_out);

  if (future_status == std::future_status::ready)
  {
    try
    {
      return future_result.get()->success;
    }
    catch (...)
    {
      return false;
    }
  }
  return false;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseDriver::on_configure(const rclcpp_lifecycle::State &state)
{
  this->activated.store(false);
  int millis;
  std::string init_service_name;
  this->get_parameter("container_name", container_name_);
  this->get_parameter("non_transmit_timeout", millis);
  this->get_parameter("node_id", this->node_id_);

  this->non_transmit_timeout_ = std::chrono::milliseconds(millis);
  init_service_name = container_name_ + "/init_driver";

  demand_init_from_master_client_ =
      this->create_client<canopen_interfaces::srv::CONode>(
          init_service_name,
          rmw_qos_profile_services_default,
          client_cbg_);
  demand_init_from_master(this->node_id_, 3s);
  if (!this->initialised_)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseDriver::on_activate(const rclcpp_lifecycle::State &state)
{
  this->activated.store(true);
  if (!this->add())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  std::this_thread::sleep_for(100ms);
  nmt_state_publisher_thread_ =
       std::thread(std::bind(&ros2_canopen::BaseDriver::nmt_listener, this));

  rpdo_publisher_thread_ =
      std::thread(std::bind(&ros2_canopen::BaseDriver::rdpo_listener, this));
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseDriver::on_deactivate(const rclcpp_lifecycle::State &state)
{
  this->activated.store(false);
  nmt_state_publisher_thread_.join();
  rpdo_publisher_thread_.join();
  if (!this->remove())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseDriver::on_cleanup(const rclcpp_lifecycle::State &state)
{
  this->activated.store(false);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BaseDriver::on_shutdown(const rclcpp_lifecycle::State &state)
{
  this->activated.store(false);
  RCLCPP_INFO(this->get_logger(), "Shutting down.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}