#ifndef CANOPEN_402_DRIVER__CANOPEN_LIFECYCLE_402_DRIVER_HPP_
#define CANOPEN_402_DRIVER__CANOPEN_LIFECYCLE_402_DRIVER_HPP_

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{
/**
 * @brief Lifecycle 402 Driver
 *
 * A very basic driver without any functionality.
 *
 */
class LifecycleCia402Driver : public ros2_canopen::LifecycleCanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>>
    node_canopen_402_driver_;

public:
  LifecycleCia402Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command()
  {
    return node_canopen_402_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command()
  {
    return node_canopen_402_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData & data)
  {
    return node_canopen_402_driver_->tpdo_transmit(data);
  }

  virtual bool sdo_write(ros2_canopen::COData & data)
  {
    return node_canopen_402_driver_->sdo_write(data);
  }

  virtual bool sdo_read(ros2_canopen::COData & data)
  {
    return node_canopen_402_driver_->sdo_read(data);
  }

  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    node_canopen_402_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    node_canopen_402_driver_->register_rpdo_cb(rpdo_cb);
  }

  double get_speed() { return node_canopen_402_driver_->get_speed(); }

  double get_position() { return node_canopen_402_driver_->get_position(); }

  bool set_target(double target) { return node_canopen_402_driver_->set_target(target); }

  bool init_motor() { return node_canopen_402_driver_->init_motor(); }

  bool recover_motor() { return node_canopen_402_driver_->recover_motor(); }

  bool halt_motor() { return node_canopen_402_driver_->halt_motor(); }

  bool set_mode_position() { return node_canopen_402_driver_->set_mode_position(); }

  bool set_mode_velocity() { return node_canopen_402_driver_->set_mode_velocity(); }

  bool set_mode_cyclic_position() { return node_canopen_402_driver_->set_mode_cyclic_position(); }

  bool set_mode_cyclic_velocity() { return node_canopen_402_driver_->set_mode_cyclic_velocity(); }

  bool set_mode_torque() { return node_canopen_402_driver_->set_mode_torque(); }

  bool set_mode_interpolated_position()
  {
    return node_canopen_402_driver_->set_mode_interpolated_position();
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_
