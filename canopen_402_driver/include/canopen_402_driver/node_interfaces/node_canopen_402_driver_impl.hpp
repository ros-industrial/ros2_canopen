//    Copyright 2023 Christoph Hellmann Santos
//                          Vishnuprasad Prachandabhanu
//                          Lovro Ivanov
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

#ifndef NODE_CANOPEN_402_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_402_DRIVER_IMPL_HPP_

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_error.hpp"

#include <optional>

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template <class NODETYPE>
NodeCanopen402Driver<NODETYPE>::NodeCanopen402Driver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::init(bool called_from_base)
{
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
  publish_joint_state =
    this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
  handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/init").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_init, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/halt").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_halt, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/recover").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_recover, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_set_mode_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/position_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_position, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_set_mode_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/velocity_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_velocity, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_set_mode_cyclic_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_velocity_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_cyclic_velocity, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_cyclic_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_position_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_cyclic_position, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_interpolated_position_service =
    this->node_->create_service<std_srvs::srv::Trigger>(
      std::string(this->node_->get_name()).append("/interpolated_position_mode").c_str(),
      std::bind(
        &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_interpolated_position, this,
        std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/torque_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_torque, this, std::placeholders::_1,
      std::placeholders::_2));

  handle_set_mode_cyclic_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_torque_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_cyclic_torque, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_target_service = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
    std::string(this->node_->get_name()).append("/target").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp::Node>::handle_set_target, this, std::placeholders::_1,
      std::placeholders::_2));
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  publish_joint_state =
    this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/init").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_init, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/halt").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_halt, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/recover").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_recover, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/position_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_position, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/velocity_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_velocity, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_cyclic_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_velocity_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_cyclic_velocity, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_cyclic_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_position_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_cyclic_position, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_interpolated_position_service = this->node_->create_service<
    std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/interpolated_position_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_interpolated_position,
      this, std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/torque_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_torque, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_mode_cyclic_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
    std::string(this->node_->get_name()).append("/cyclic_torque_mode").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_cyclic_torque, this,
      std::placeholders::_1, std::placeholders::_2));

  handle_set_target_service = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
    std::string(this->node_->get_name()).append("/target").c_str(),
    std::bind(
      &NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_target, this,
      std::placeholders::_1, std::placeholders::_2));
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::configure(false);
  std::optional<double> scale_pos_to_dev;
  std::optional<double> scale_pos_from_dev;
  std::optional<double> scale_vel_to_dev;
  std::optional<double> scale_vel_from_dev;
  std::optional<double> offset_pos_to_dev;
  std::optional<double> offset_pos_from_dev;
  std::optional<int> switching_state;
  try
  {
    scale_pos_to_dev = std::optional(this->config_["scale_pos_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_pos_from_dev = std::optional(this->config_["scale_pos_from_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_vel_to_dev = std::optional(this->config_["scale_vel_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_vel_from_dev = std::optional(this->config_["scale_vel_from_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    offset_pos_to_dev = std::optional(this->config_["offset_pos_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    offset_pos_from_dev = std::optional(this->config_["offset_from_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    switching_state = std::optional(this->config_["switching_state"].as<int>());
  }
  catch (...)
  {
  }

  // auto period = this->config_["scale_eff_to_dev"].as<double>();
  // auto period = this->config_["scale_eff_from_dev"].as<double>();
  scale_pos_to_dev_ = scale_pos_to_dev.value_or(1000.0);
  scale_pos_from_dev_ = scale_pos_from_dev.value_or(0.001);
  scale_vel_to_dev_ = scale_vel_to_dev.value_or(1000.0);
  scale_vel_from_dev_ = scale_vel_from_dev.value_or(0.001);
  offset_pos_to_dev_ = offset_pos_to_dev.value_or(0.0);
  offset_pos_from_dev_ = offset_pos_from_dev.value_or(0.0);
  switching_state_ = (ros2_canopen::State402::InternalState)switching_state.value_or(
    (int)ros2_canopen::State402::InternalState::Operation_Enable);
  RCLCPP_INFO(
    this->node_->get_logger(),
    "scale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ %f\nscale_vel_from_dev_ "
    "%f\noffset_pos_to_dev_ %f\noffset_pos_from_dev_ %f\n",
    scale_pos_to_dev_, scale_pos_from_dev_, scale_vel_to_dev_, scale_vel_from_dev_,
    offset_pos_to_dev_, offset_pos_from_dev_);
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
  std::optional<double> scale_pos_to_dev;
  std::optional<double> scale_pos_from_dev;
  std::optional<double> scale_vel_to_dev;
  std::optional<double> scale_vel_from_dev;
  std::optional<double> offset_pos_to_dev;
  std::optional<double> offset_pos_from_dev;
  std::optional<int> switching_state;
  try
  {
    scale_pos_to_dev = std::optional(this->config_["scale_pos_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_pos_from_dev = std::optional(this->config_["scale_pos_from_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_vel_to_dev = std::optional(this->config_["scale_vel_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    scale_vel_from_dev = std::optional(this->config_["scale_vel_from_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    offset_pos_to_dev = std::optional(this->config_["offset_pos_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    offset_pos_from_dev = std::optional(this->config_["offset_from_to_dev"].as<double>());
  }
  catch (...)
  {
  }
  try
  {
    switching_state = std::optional(this->config_["switching_state"].as<int>());
  }
  catch (...)
  {
  }

  // auto period = this->config_["scale_eff_to_dev"].as<double>();
  // auto period = this->config_["scale_eff_from_dev"].as<double>();
  scale_pos_to_dev_ = scale_pos_to_dev.value_or(1000.0);
  scale_pos_from_dev_ = scale_pos_from_dev.value_or(0.001);
  scale_vel_to_dev_ = scale_vel_to_dev.value_or(1000.0);
  scale_vel_from_dev_ = scale_vel_from_dev.value_or(0.001);
  offset_pos_to_dev_ = offset_pos_to_dev.value_or(0.0);
  offset_pos_from_dev_ = offset_pos_from_dev.value_or(0.0);
  switching_state_ = (ros2_canopen::State402::InternalState)switching_state.value_or(
    (int)ros2_canopen::State402::InternalState::Operation_Enable);
  RCLCPP_INFO(
    this->node_->get_logger(),
    "scale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ %f\nscale_vel_from_dev_ "
    "%f\noffset_pos_to_dev_ %f\noffset_pos_from_dev_ %f\n",
    scale_pos_to_dev_, scale_pos_from_dev_, scale_vel_to_dev_, scale_vel_from_dev_,
    offset_pos_to_dev_, offset_pos_from_dev_);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  motor_->registerDefaultModes();
  motor_->set_diagnostic_status_msgs(this->diagnostic_collector_, this->diagnostic_enabled_);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::deactivate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  timer_->cancel();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  motor_->handleRead();
  motor_->handleWrite();
  publish();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::publish()
{
  sensor_msgs::msg::JointState js_msg;
  js_msg.name.push_back(this->node_->get_name());
  js_msg.position.push_back(motor_->get_position() * scale_pos_from_dev_ + offset_pos_from_dev_);
  js_msg.velocity.push_back(motor_->get_speed() * scale_vel_from_dev_);
  js_msg.effort.push_back(0.0);
  publish_joint_state->publish(js_msg);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  motor_ = std::make_shared<Motor402>(this->lely_driver_, switching_state_);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_init(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load())
  {
    bool temp = motor_->handleInit();
    response->success = temp;
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_recover(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load())
  {
    response->success = motor_->handleRecover();
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_halt(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load())
  {
    response->success = motor_->handleHalt();
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Profiled_Position);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Profiled_Velocity);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Position);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_interpolated_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Interpolated_Position);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Velocity);
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Profiled_Torque);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Torque);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_target(
  const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
  canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
  if (this->activated_.load())
  {
    auto mode = motor_->getMode();
    double target;
    if (
      (mode == MotorBase::Profiled_Position) or (mode == MotorBase::Cyclic_Synchronous_Position) or
      (mode == MotorBase::Interpolated_Position))
    {
      target = request->target * scale_pos_to_dev_ + offset_pos_to_dev_;
    }
    else if (
      (mode == MotorBase::Velocity) or (mode == MotorBase::Profiled_Velocity) or
      (mode == MotorBase::Cyclic_Synchronous_Velocity))
    {
      target = request->target * scale_vel_to_dev_;
    }
    else
    {
      target = request->target;
    }

    response->success = motor_->setTarget(target);
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::init_motor()
{
  if (this->activated_.load())
  {
    bool temp = motor_->handleInit();
    return temp;
  }
  else
  {
    RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::recover_motor()
{
  if (this->activated_.load())
  {
    return motor_->handleRecover();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::halt_motor()
{
  if (this->activated_.load())
  {
    return motor_->handleHalt();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_operation_mode(uint16_t mode)
{
  if (this->activated_.load())
  {
    if (motor_->getMode() != mode)
    {
      return motor_->enterModeAndWait(mode);
    }
    else
    {
      return false;
    }
  }
  return false;
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_target(double target)
{
  if (this->activated_.load())
  {
    auto mode = motor_->getMode();
    double scaled_target;
    if (
      (mode == MotorBase::Profiled_Position) or (mode == MotorBase::Cyclic_Synchronous_Position) or
      (mode == MotorBase::Interpolated_Position))
    {
      scaled_target = target * scale_pos_to_dev_ + offset_pos_to_dev_;
    }
    else if (
      (mode == MotorBase::Velocity) or (mode == MotorBase::Profiled_Velocity) or
      (mode == MotorBase::Cyclic_Synchronous_Velocity))
    {
      scaled_target = target * scale_vel_to_dev_;
    }
    else
    {
      scaled_target = target;
    }
    // RCLCPP_INFO(this->node_->get_logger(), "Scaled target %f", scaled_target);
    return motor_->setTarget(scaled_target);
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::diagnostic_callback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  this->motor_->handleDiag();

  stat.summary(this->diagnostic_collector_->getLevel(), this->diagnostic_collector_->getMessage());
  stat.add("device_state", this->diagnostic_collector_->getValue("DEVICE"));
  stat.add("nmt_state", this->diagnostic_collector_->getValue("NMT"));
  stat.add("emcy_state", this->diagnostic_collector_->getValue("EMCY"));
  stat.add("cia402_mode", this->diagnostic_collector_->getValue("cia402_mode"));
  stat.add("cia402_state", this->diagnostic_collector_->getValue("cia402_state"));
}

#endif
