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

#ifndef NODE_CANOPEN_402_DRIVER
#define NODE_CANOPEN_402_DRIVER

#include <cstdint>
#include "canopen_402_driver/motor.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopen402Driver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
      std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::vector<std::shared_ptr<Motor402>> motors_;
  uint8_t num_channels_;
  std::vector<std::string> channel_names_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Per-channel service handles (Option B: one service per channel)
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_init_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_enable_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_disable_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_halt_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_recover_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_position_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_torque_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_velocity_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_cyclic_velocity_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_cyclic_position_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_cyclic_torque_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>
    handle_set_mode_interpolated_position_services_;
  std::vector<rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr>
    handle_set_target_services_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publish_joint_state;

  // Global scale/offset (used as defaults)
  double scale_pos_to_dev_;
  double scale_pos_from_dev_;
  double scale_vel_to_dev_;
  double scale_vel_from_dev_;
  double scale_eff_from_dev_;
  double offset_pos_to_dev_;
  double offset_pos_from_dev_;

  // Per-channel scale/offset (optional overrides)
  std::vector<double> channel_scale_pos_to_dev_;
  std::vector<double> channel_scale_pos_from_dev_;
  std::vector<double> channel_scale_vel_to_dev_;
  std::vector<double> channel_scale_vel_from_dev_;
  std::vector<double> channel_scale_eff_from_dev_;
  std::vector<double> channel_offset_pos_to_dev_;
  std::vector<double> channel_offset_pos_from_dev_;

  ros2_canopen::State402::InternalState switching_state_;
  int homing_timeout_seconds_;

  void publish();
  virtual void poll_timer_callback() override;
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

  // Helper to create per-channel services after configure
  void create_per_channel_services();

public:
  NodeCanopen402Driver(NODETYPE * node);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;

  virtual double get_effort(uint8_t channel = 0)
  {
    if (channel >= motors_.size()) return 0.0;
    double scale = (channel < channel_scale_eff_from_dev_.size())
                     ? channel_scale_eff_from_dev_[channel]
                     : scale_eff_from_dev_;
    return motors_[channel]->get_effort() * scale;
  }

  virtual double get_speed(uint8_t channel = 0)
  {
    if (channel >= motors_.size()) return 0.0;
    double scale = (channel < channel_scale_vel_from_dev_.size())
                     ? channel_scale_vel_from_dev_[channel]
                     : scale_vel_from_dev_;
    return motors_[channel]->get_speed() * scale;
  }

  virtual double get_position(uint8_t channel = 0)
  {
    if (channel >= motors_.size()) return 0.0;
    double scale = (channel < channel_scale_pos_from_dev_.size())
                     ? channel_scale_pos_from_dev_[channel]
                     : scale_pos_from_dev_;
    double offset = (channel < channel_offset_pos_from_dev_.size())
                      ? channel_offset_pos_from_dev_[channel]
                      : offset_pos_from_dev_;
    return motors_[channel]->get_position() * scale + offset;
  }

  virtual uint16_t get_mode(uint8_t channel = 0)
  {
    if (channel >= motors_.size()) return 0;
    return motors_[channel]->getMode();
  }

  /**
   * @brief Service Callback to initialise device
   *
   * Calls Motor402::handleInit function. Brings motor to enabled
   * state and homes it.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to enable device
   *
   * Calls Motor402::handleEnable function. Brings motor to enabled
   * state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_enable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to disable device
   *
   * Calls Motor402::handleDisable function. Brings motor to switched on
   * disabled state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_disable(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Method to initialise device
   *
   * Calls Motor402::handleInit function. Brings motor to enabled
   * state and homes it.
   *
   * @param [in] void
   *
   * @return  bool
   * Indicates initialisation procedure result
   */
  bool init_motor(uint8_t channel = 0);

  /**
   * @brief Service Callback to recover device
   *
   * Calls Motor402::handleRecover function. Resets faults and brings
   * motor to enabled state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Method to recover device
   *
   * Calls Motor402::handleRecover function. Resets faults and brings
   * motor to enabled state.
   *
   * @param [in] void
   *
   * @return bool
   */
  bool recover_motor(uint8_t channel = 0);

  /**
   * @brief Service Callback to halt device
   *
   * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
   * Motor state depends on devices configuration specifically object
   * 0x605A.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Method to halt device
   *
   * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
   * Motor state depends on devices configuration specifically object
   * 0x605A.
   *
   * @param [in] void
   *
   * @return bool
   */
  bool halt_motor(uint8_t channel = 0);

  /**
   * @brief Service Callback to set operation mode
   *
   * Calls Motor402::enterModeAndWait with requested Operation Mode.
   *
   * @param [in] request Requested Operation Mode as MotorBase::Profiled_Position or
   * MotorBase::Profiled_Velocity or MotorBase::Profiled_Torque or MotorBase::Cyclic_Position or
   * MotorBase::Cyclic_Velocity or MotorBase::Cyclic_Torque or MotorBase::Interpolated_Position
   * @param [out] response
   */
  bool set_operation_mode(uint16_t mode, uint8_t channel = 0);

  /**
   * @brief Service Callback to set profiled position mode
   *
   * Calls Motor402::enterModeAndWait with Profiled Position Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Profiled Position Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set profiled velocity mode
   *
   * Calls Motor402::enterModeAndWait with Profiled Velocity Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Profiled Velocity Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set cyclic position mode
   *
   * Calls Motor402::enterModeAndWait with Cyclic Position Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Cyclic Position Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_cyclic_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set interpolated position mode
   *
   * Calls Motor402::enterModeAndWait with Interpolated Position Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Interpolated Position Mode. This only supports linear mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_interpolated_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set cyclic velocity mode
   *
   * Calls Motor402::enterModeAndWait with Cyclic Velocity Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Cyclic Velocity Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_cyclic_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set profiled torque mode
   *
   * Calls Motor402::enterModeAndWait with Profiled Torque Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Profiled Torque Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set cyclic torque mode
   *
   * Calls Motor402::enterModeAndWait with Cyclic Torque Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to Cyclic Torque Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_mode_cyclic_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel = 0);

  /**
   * @brief Service Callback to set target
   *
   * Calls Motor402::setTarget and sets the requested target value. Note
   * that the resulting movement is dependent on the Operation Mode and the
   * drives state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response,
    const uint8_t channel = 0);

  /**
   * @brief Method to set target
   *
   * Calls Motor402::setTarget and sets the requested target value. Note
   * that the resulting movement is dependent on the Operation Mode and the
   * drives state.
   *
   * @param [in] double target value
   * @param [in] uint8_t channel (default 0 for backward compatibility)
   *
   * @return bool
   */
  bool set_target(double target, uint8_t channel = 0);
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
