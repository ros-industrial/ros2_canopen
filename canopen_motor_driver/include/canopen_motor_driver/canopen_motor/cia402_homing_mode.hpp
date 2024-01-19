// Copyright 2024 ROS-Industrial
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_HOMING_MODE_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_HOMING_MODE_HPP_
#include <future>
#include <memory>
#include "cia402_data.hpp"
#include "cia402_mode.hpp"
#include "lely/coapp/logical_driver.hpp"

namespace ros2_canopen
{
class HomingMode : public OperationMode
{
protected:
  enum homing_states
  {
    NO_HOME,
    HOMING_NEW,
    HOMING_ACTIVE,
    HOMING_ATTAINED,
  };
  homing_states homing_state_;

public:
  HomingMode(int8_t num, std::shared_ptr<CiA402Data> data)
  : OperationMode(num, data), homing_state_(NO_HOME)
  {
  }

  virtual void start(
    lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver> * driver) override
  {
    OperationMode::start(driver);
  }

  virtual void stop() override { OperationMode::stop(); }

  /**
   * @brief Fetch data from device available after boot.
   *
   */
  virtual void on_boot(lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver> & driver) {}

  /**
   * @brief Do checks with data from device available after boot.
   *
   * @return true
   * @return false
   */
  virtual bool on_boot_check() { return (data_->get_supported_modes() >> (num_ - 1)) & 0x1; }

  /**
   * @brief Get the target state
   *
   * @return true
   * @return false
   */
  virtual bool is_target_active()
  {
    if (homing_state_ == HOMING_ACTIVE)
    {
      return true;
    }
    return false;
  }

  bool is_homing_attained()
  {
    if (homing_state_ == HOMING_ATTAINED)
    {
      return true;
    }
    return false;
  }

  /**
   * @brief Set a new target
   *
   * This function will start the actual homing process.
   * @param target
   */
  virtual void set_target(double target = 0.0) override
  {
    if (
      started_ && data_->get_status_word_bit(SW_TARGET_REACHED_MASK) &&
      !data_->get_status_word_bit(SW_OPERATION_MODE1_MASK))
    {
      homing_state_ = HOMING_NEW;
      data_->set_control_word_bit(CW_OPERATION_MODE1_MASK, true);
    }
  }

  /**
   * @brief Store data from RPDO if necessary.
   *
   * This gets called on every RPDO message.
   *
   * @param idx
   * @param subidx
   * @param driver
   */
  virtual void on_rpdo(uint16_t idx, uint8_t subidx) {}

  /**
   * @brief Sync callback.
   *
   * In this callback the periodic work should be done.
   *
   * @param driver
   */
  virtual void on_sync()
  {
    bool tr = data_->get_status_word_bit(SW_TARGET_REACHED_MASK);
    bool ha = data_->get_status_word_bit(SW_OPERATION_MODE1_MASK);
    bool he = data_->get_status_word_bit(SW_OPERATION_MODE2_MASK);
    if (homing_state_ == HOMING_NEW)
    {
      homing_state_ = HOMING_ACTIVE;
      return;
    }
    if (tr && ha)
    {
      homing_state_ = HOMING_ATTAINED;
      data_->set_control_word_bit(CW_OPERATION_MODE1_MASK, false);
      data_->set_control_word_bit(CW_OPERATION_MODE2_MASK, false);
      data_->set_control_word_bit(CW_OPERATION_MODE3_MASK, false);
    }
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_HOMING_MODE_HPP
