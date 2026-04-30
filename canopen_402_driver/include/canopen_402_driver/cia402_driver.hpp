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

#ifndef CANOPEN_402_DRIVER__402_DRIVER_HPP_
#define CANOPEN_402_DRIVER__402_DRIVER_HPP_
#include <cstdint>
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{
/**
 * @brief Abstract Class for a CANopen Device Node
 *
 * This class provides the base functionality for creating a
 * CANopen device node. It provides callbacks for nmt and rpdo.
 */
class Cia402Driver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopen402Driver<rclcpp::Node>> node_canopen_402_driver_;

public:
  Cia402Driver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

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

  double get_effort(uint8_t channel = 0) { return node_canopen_402_driver_->get_effort(channel); }

  double get_speed(uint8_t channel = 0) { return node_canopen_402_driver_->get_speed(channel); }

  double get_position(uint8_t channel = 0)
  {
    return node_canopen_402_driver_->get_position(channel);
  }

  bool set_target(double target, uint8_t channel = 0)
  {
    return node_canopen_402_driver_->set_target(target, channel);
  }

  bool init_motor(uint8_t channel = 0) { return node_canopen_402_driver_->init_motor(channel); }

  bool recover_motor(uint8_t channel = 0)
  {
    return node_canopen_402_driver_->recover_motor(channel);
  }

  bool halt_motor(uint8_t channel = 0) { return node_canopen_402_driver_->halt_motor(channel); }

  uint16_t get_mode(uint8_t channel = 0) { return node_canopen_402_driver_->get_mode(channel); }

  bool set_operation_mode(uint16_t mode, uint8_t channel = 0)
  {
    return node_canopen_402_driver_->set_operation_mode(mode, channel);
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_
