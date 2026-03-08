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

#include <sys/types.h>
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
  // Per-channel services will be created in configure() after num_channels_ is known
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
  publish_joint_state =
    this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
  // Per-channel services will be created in configure() after num_channels_ is known
}

// Helper method to create per-channel services (called from configure)
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::create_per_channel_services()
{
  // Ensure per-channel storage exists
  channels_.resize(num_channels_);

  // Create services for each channel
  for (uint8_t ch = 0; ch < num_channels_; ++ch)
  {
    // Service naming:
    // - Single-channel (num_channels_ == 1): Use legacy names like "init", "target", etc.
    // - Multi-channel: Use channel_name as prefix like "channel_name/init"
    std::string service_prefix;
    if (num_channels_ == 1)
    {
      // Single-channel: legacy behavior
      service_prefix = std::string(this->node_->get_name()) + "/";
    }
    else
    {
      // Multi-channel: use channel_name as prefix
      if (ch < channel_names_.size())
      {
        service_prefix = channel_names_[ch] + "/";
      }
      else
      {
        // Fallback if channel_names not configured
        service_prefix = "ch" + std::to_string(ch) + "/";
      }
    }

    // Init service
    channels_[ch].handle_init = this->node_->template create_service<std_srvs::srv::Trigger>(
      service_prefix + "init", [this, ch](
                                 const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response)
      { this->handle_init(request, response, ch); });

    // Enable service
    channels_[ch].handle_enable = this->node_->template create_service<std_srvs::srv::Trigger>(
      service_prefix + "enable", [this, ch](
                                   const std_srvs::srv::Trigger::Request::SharedPtr request,
                                   std_srvs::srv::Trigger::Response::SharedPtr response)
      { this->handle_enable(request, response, ch); });

    // Disable service
    channels_[ch].handle_disable = this->node_->template create_service<std_srvs::srv::Trigger>(
      service_prefix + "disable", [this, ch](
                                    const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response)
      { this->handle_disable(request, response, ch); });

    // Halt service
    channels_[ch].handle_halt = this->node_->template create_service<std_srvs::srv::Trigger>(
      service_prefix + "halt", [this, ch](
                                 const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response)
      { this->handle_halt(request, response, ch); });

    // Recover service
    channels_[ch].handle_recover = this->node_->template create_service<std_srvs::srv::Trigger>(
      service_prefix + "recover", [this, ch](
                                    const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response)
      { this->handle_recover(request, response, ch); });

    // Position mode service
    channels_[ch].handle_set_mode_position =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "position_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_position(request, response, ch); });

    // Velocity mode service
    channels_[ch].handle_set_mode_velocity =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "velocity_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_velocity(request, response, ch); });

    // Torque mode service
    channels_[ch].handle_set_mode_torque =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "torque_mode", [this, ch](
                                          const std_srvs::srv::Trigger::Request::SharedPtr request,
                                          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_torque(request, response, ch); });

    // Cyclic velocity mode service
    channels_[ch].handle_set_mode_cyclic_velocity =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "cyclic_velocity_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_cyclic_velocity(request, response, ch); });

    // Cyclic position mode service
    channels_[ch].handle_set_mode_cyclic_position =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "cyclic_position_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_cyclic_position(request, response, ch); });

    // Cyclic torque mode service
    channels_[ch].handle_set_mode_cyclic_torque =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "cyclic_torque_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_cyclic_torque(request, response, ch); });

    // Interpolated position mode service
    channels_[ch].handle_set_mode_interpolated_position =
      this->node_->template create_service<std_srvs::srv::Trigger>(
        service_prefix + "interpolated_position_mode",
        [this, ch](
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
        { this->handle_set_mode_interpolated_position(request, response, ch); });

    // Target service (uses COTargetDouble without channel field, since channel is in service name)
    channels_[ch].handle_set_target =
      this->node_->template create_service<canopen_interfaces::srv::COTargetDouble>(
        service_prefix + "target",
        [this, ch](
          const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
          canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
        { response->success = this->set_target(request->target, ch); });
  }
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::configure(false);
  std::optional<double> scale_pos_to_dev;
  std::optional<double> scale_pos_from_dev;
  std::optional<double> scale_vel_to_dev;
  std::optional<double> scale_vel_from_dev;
  std::optional<double> scale_eff_from_dev;
  std::optional<double> offset_pos_to_dev;
  std::optional<double> offset_pos_from_dev;
  std::optional<int> switching_state;
  std::optional<int> homing_timeout_seconds;
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
    scale_eff_from_dev = std::optional(this->config_["scale_eff_from_dev"].as<double>());
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
    offset_pos_from_dev = std::optional(this->config_["offset_pos_from_dev"].as<double>());
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
  try
  {
    homing_timeout_seconds = std::optional(this->config_["homing_timeout_seconds"].as<int>());
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
  scale_eff_from_dev_ = scale_eff_from_dev.value_or(0.001);
  offset_pos_to_dev_ = offset_pos_to_dev.value_or(0.0);
  offset_pos_from_dev_ = offset_pos_from_dev.value_or(0.0);
  switching_state_ = (ros2_canopen::State402::InternalState)switching_state.value_or(
    (int)ros2_canopen::State402::InternalState::Operation_Enable);
  homing_timeout_seconds_ = homing_timeout_seconds.value_or(10);

  // Multi-channel configuration
  num_channels_ = 1;
  try
  {
    num_channels_ = this->config_["num_channels"].as<uint8_t>();
  }
  catch (...)
  {
  }

  // Parse channel names
  channel_names_.clear();
  try
  {
    auto names_node = this->config_["channel_names"];
    if (names_node.IsDefined() && names_node.IsSequence())
    {
      for (size_t i = 0; i < names_node.size(); ++i)
      {
        channel_names_.push_back(names_node[i].as<std::string>());
      }
    }
  }
  catch (...)
  {
  }

  // Generate default names if not provided
  if (channel_names_.empty())
  {
    for (uint8_t i = 0; i < num_channels_; ++i)
    {
      channel_names_.push_back(std::string(this->node_->get_name()) + "/" + std::to_string(i));
    }
  }

  // Resolve per-channel scales/offsets into per-channel contexts
  channels_.clear();
  channels_.resize(num_channels_);
  for (uint8_t i = 0; i < num_channels_; ++i)
  {
    channels_[i].scale_pos_to_dev = scale_pos_to_dev_;
    channels_[i].scale_pos_from_dev = scale_pos_from_dev_;
    channels_[i].scale_vel_to_dev = scale_vel_to_dev_;
    channels_[i].scale_vel_from_dev = scale_vel_from_dev_;
    channels_[i].scale_eff_from_dev = scale_eff_from_dev_;
    channels_[i].offset_pos_to_dev = offset_pos_to_dev_;
    channels_[i].offset_pos_from_dev = offset_pos_from_dev_;
  }

  try
  {
    auto channels_node = this->config_["channels"];
    if (channels_node.IsDefined() && channels_node.IsSequence())
    {
      for (size_t i = 0; i < channels_node.size() && i < num_channels_; ++i)
      {
        auto ch = channels_node[i];
        try
        {
          channels_[i].scale_pos_to_dev = ch["scale_pos_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_pos_from_dev = ch["scale_pos_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_vel_to_dev = ch["scale_vel_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_vel_from_dev = ch["scale_vel_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_eff_from_dev = ch["scale_eff_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].offset_pos_to_dev = ch["offset_pos_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].offset_pos_from_dev = ch["offset_pos_from_dev"].as<double>();
        }
        catch (...)
        {
        }
      }
    }
  }
  catch (...)
  {
  }

  RCLCPP_INFO(
    this->node_->get_logger(),
    "num_channels: %u\nscale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ "
    "%f\nscale_vel_from_dev_ "
    "%f\nscale_eff_from_dev_ %f\noffset_pos_to_dev_ %f\noffset_pos_from_dev_ "
    "%f\nhoming_timeout_seconds_ %i\n",
    num_channels_, scale_pos_to_dev_, scale_pos_from_dev_, scale_vel_to_dev_, scale_vel_from_dev_,
    scale_eff_from_dev_, offset_pos_to_dev_, offset_pos_from_dev_, homing_timeout_seconds_);

  // Create per-channel services
  create_per_channel_services();
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
  std::optional<double> scale_pos_to_dev;
  std::optional<double> scale_pos_from_dev;
  std::optional<double> scale_vel_to_dev;
  std::optional<double> scale_vel_from_dev;
  std::optional<double> scale_eff_from_dev;
  std::optional<double> offset_pos_to_dev;
  std::optional<double> offset_pos_from_dev;
  std::optional<int> switching_state;
  std::optional<int> homing_timeout_seconds;
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
    scale_eff_from_dev = std::optional(this->config_["scale_eff_from_dev"].as<double>());
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
    offset_pos_from_dev = std::optional(this->config_["offset_pos_from_dev"].as<double>());
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
  try
  {
    homing_timeout_seconds = std::optional(this->config_["homing_timeout_seconds"].as<int>());
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
  scale_eff_from_dev_ = scale_eff_from_dev.value_or(0.001);
  offset_pos_to_dev_ = offset_pos_to_dev.value_or(0.0);
  offset_pos_from_dev_ = offset_pos_from_dev.value_or(0.0);
  switching_state_ = (ros2_canopen::State402::InternalState)switching_state.value_or(
    (int)ros2_canopen::State402::InternalState::Operation_Enable);
  homing_timeout_seconds_ = homing_timeout_seconds.value_or(10);

  // Multi-channel configuration
  num_channels_ = 1;
  try
  {
    num_channels_ = this->config_["num_channels"].as<uint8_t>();
  }
  catch (...)
  {
  }

  // Parse channel names
  channel_names_.clear();
  try
  {
    auto names_node = this->config_["channel_names"];
    if (names_node.IsDefined() && names_node.IsSequence())
    {
      for (size_t i = 0; i < names_node.size(); ++i)
      {
        channel_names_.push_back(names_node[i].as<std::string>());
      }
    }
  }
  catch (...)
  {
  }

  // Generate default names if not provided
  if (channel_names_.empty())
  {
    for (uint8_t i = 0; i < num_channels_; ++i)
    {
      channel_names_.push_back(std::string(this->node_->get_name()) + "/" + std::to_string(i));
    }
  }

  // Resolve per-channel scales/offsets into per-channel contexts
  channels_.clear();
  channels_.resize(num_channels_);
  for (uint8_t i = 0; i < num_channels_; ++i)
  {
    channels_[i].scale_pos_to_dev = scale_pos_to_dev_;
    channels_[i].scale_pos_from_dev = scale_pos_from_dev_;
    channels_[i].scale_vel_to_dev = scale_vel_to_dev_;
    channels_[i].scale_vel_from_dev = scale_vel_from_dev_;
    channels_[i].scale_eff_from_dev = scale_eff_from_dev_;
    channels_[i].offset_pos_to_dev = offset_pos_to_dev_;
    channels_[i].offset_pos_from_dev = offset_pos_from_dev_;
  }

  try
  {
    auto channels_node = this->config_["channels"];
    if (channels_node.IsDefined() && channels_node.IsSequence())
    {
      for (size_t i = 0; i < channels_node.size() && i < num_channels_; ++i)
      {
        auto ch = channels_node[i];
        try
        {
          channels_[i].scale_pos_to_dev = ch["scale_pos_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_pos_from_dev = ch["scale_pos_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_vel_to_dev = ch["scale_vel_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_vel_from_dev = ch["scale_vel_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].scale_eff_from_dev = ch["scale_eff_from_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].offset_pos_to_dev = ch["offset_pos_to_dev"].as<double>();
        }
        catch (...)
        {
        }
        try
        {
          channels_[i].offset_pos_from_dev = ch["offset_pos_from_dev"].as<double>();
        }
        catch (...)
        {
        }
      }
    }
  }
  catch (...)
  {
  }

  RCLCPP_INFO(
    this->node_->get_logger(),
    "num_channels: %u\nscale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ "
    "%f\nscale_vel_from_dev_ "
    "%f\nscale_eff_from_dev_ %f\noffset_pos_to_dev_ %f\noffset_pos_from_dev_ "
    "%f\nhoming_timeout_seconds_ %i\n",
    num_channels_, scale_pos_to_dev_, scale_pos_from_dev_, scale_vel_to_dev_, scale_vel_from_dev_,
    scale_eff_from_dev_, offset_pos_to_dev_, offset_pos_from_dev_, homing_timeout_seconds_);

  // Create per-channel services
  create_per_channel_services();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  // Register default modes for all motor instances
  for (auto & ch : channels_)
  {
    if (!ch.motor) continue;
    ch.motor->registerDefaultModes();
    ch.motor->set_diagnostic_status_msgs(this->diagnostic_collector_, this->diagnostic_enabled_);
  }
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
  // Handle read/write for all motors
  for (auto & ch : channels_)
  {
    if (!ch.motor) continue;
    ch.motor->handleRead();
    ch.motor->handleWrite();
  }
  publish();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::publish()
{
  sensor_msgs::msg::JointState js_msg;
  // Publish joint state for all channels
  for (size_t ch = 0; ch < channels_.size(); ++ch)
  {
    std::string name;
    if (num_channels_ == 1)
    {
      // Single-channel: use node name (legacy behavior)
      name = std::string(this->node_->get_name());
    }
    else
    {
      // Multi-channel: use channel_name
      if (ch < channel_names_.size())
      {
        name = channel_names_[ch];
      }
      else
      {
        // Fallback if channel_names not configured
        name = std::string(this->node_->get_name()) + "/ch" + std::to_string(ch);
      }
    }

    js_msg.name.push_back(name);
    if (!channels_[ch].motor)
    {
      js_msg.position.push_back(0.0);
      js_msg.velocity.push_back(0.0);
      js_msg.effort.push_back(0.0);
      continue;
    }

    js_msg.position.push_back(
      channels_[ch].motor->get_position() * channels_[ch].scale_pos_from_dev +
      channels_[ch].offset_pos_from_dev);
    js_msg.velocity.push_back(channels_[ch].motor->get_speed() * channels_[ch].scale_vel_from_dev);
    js_msg.effort.push_back(channels_[ch].motor->get_effort() * channels_[ch].scale_eff_from_dev);
  }
  publish_joint_state->publish(js_msg);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  // Create separate Motor402 instance for each channel
  channels_.resize(num_channels_);
  for (uint8_t ch = 0; ch < num_channels_; ++ch)
  {
    channels_[ch].motor =
      std::make_shared<Motor402>(this->lely_driver_, switching_state_, homing_timeout_seconds_, ch);
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_init(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = this->init_motor(channel);
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_recover(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = this->recover_motor(channel);
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_halt(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = this->halt_motor(channel);
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Profiled_Position, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Profiled_Velocity, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Position, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_interpolated_position(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Interpolated_Position, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_velocity(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Velocity, channel);
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Profiled_Torque, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_torque(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_operation_mode(MotorBase::Cyclic_Synchronous_Torque, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_target(
  const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
  canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response, const uint8_t channel)
{
  response->success = set_target(request->target, channel);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_disable(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size() || !channels_[channel].motor)
    {
      response->success = false;
      return;
    }
    response->success = channels_[channel].motor->handleDisable();
  }
  else
  {
    response->success = false;
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_enable(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response, const uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size() || !channels_[channel].motor)
    {
      response->success = false;
      return;
    }
    response->success = channels_[channel].motor->handleEnable();
  }
  else
  {
    response->success = false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::init_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size())
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "Invalid channel %u (max %lu)", channel, channels_.size() - 1);
      return false;
    }
    if (!channels_[channel].motor) return false;
    return channels_[channel].motor->handleInit();
  }
  else
  {
    RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::recover_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size())
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "Invalid channel %u (max %lu)", channel, channels_.size() - 1);
      return false;
    }
    if (!channels_[channel].motor) return false;
    return channels_[channel].motor->handleRecover();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::halt_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size())
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "Invalid channel %u (max %lu)", channel, channels_.size() - 1);
      return false;
    }
    if (!channels_[channel].motor) return false;
    return channels_[channel].motor->handleHalt();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_operation_mode(uint16_t mode, uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size())
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "Invalid channel %u (max %lu)", channel, channels_.size() - 1);
      return false;
    }

    if (!channels_[channel].motor) return false;

    if (channels_[channel].motor->getMode() != mode)
    {
      return channels_[channel].motor->enterModeAndWait(mode);
    }
    else
    {
      return false;
    }
  }
  return false;
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_target(double target, uint8_t channel)
{
  if (this->activated_.load())
  {
    if (channel >= channels_.size())
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "Invalid channel %u (max %lu)", channel, channels_.size() - 1);
      return false;
    }

    if (!channels_[channel].motor) return false;

    auto mode = channels_[channel].motor->getMode();

    const double scale_pos_to_dev = channels_[channel].scale_pos_to_dev;
    const double offset_pos_to_dev = channels_[channel].offset_pos_to_dev;
    const double scale_vel_to_dev = channels_[channel].scale_vel_to_dev;

    double scaled_target;
    if (
      (mode == MotorBase::Profiled_Position) or (mode == MotorBase::Cyclic_Synchronous_Position) or
      (mode == MotorBase::Interpolated_Position))
    {
      scaled_target = target * scale_pos_to_dev + offset_pos_to_dev;
    }
    else if (
      (mode == MotorBase::Velocity) or (mode == MotorBase::Profiled_Velocity) or
      (mode == MotorBase::Cyclic_Synchronous_Velocity))
    {
      scaled_target = target * scale_vel_to_dev;
    }
    else
    {
      scaled_target = target;
    }

    return channels_[channel].motor->setTarget(scaled_target);
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
  // Handle diagnostics for all motors
  for (size_t ch = 0; ch < channels_.size(); ++ch)
  {
    if (!channels_[ch].motor) continue;
    channels_[ch].motor->handleDiag();
  }

  stat.summary(this->diagnostic_collector_->getLevel(), this->diagnostic_collector_->getMessage());
  stat.add("device_state", this->diagnostic_collector_->getValue("DEVICE"));
  stat.add("nmt_state", this->diagnostic_collector_->getValue("NMT"));
  stat.add("emcy_state", this->diagnostic_collector_->getValue("EMCY"));
  stat.add("cia402_mode", this->diagnostic_collector_->getValue("cia402_mode"));
  stat.add("cia402_state", this->diagnostic_collector_->getValue("cia402_state"));

  // Add per-channel diagnostic info
  for (size_t ch = 0; ch < channels_.size(); ++ch)
  {
    std::string ch_prefix = "ch" + std::to_string(ch) + "_";
    stat.add(
      ch_prefix + "mode",
      this->diagnostic_collector_->getValue((ch_prefix + "cia402_mode").c_str()));
    stat.add(
      ch_prefix + "state",
      this->diagnostic_collector_->getValue((ch_prefix + "cia402_state").c_str()));
  }
}

#endif
