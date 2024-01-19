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

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_MODE_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_MODE_HPP_
#include <future>
#include <memory>
#include "cia402_data.hpp"
#include "lely/coapp/logical_driver.hpp"

namespace ros2_canopen
{
class OperationMode
{
protected:
  enum target_states
  {
    NEW_TARGET,
    TARGET_ACTIVE,
    TARGET_REACHED
  };
  target_states target_state_;
  const int8_t num_;
  std::shared_ptr<CiA402Data> data_;
  double target_;
  lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver> * driver_;
  bool started_;

public:
  OperationMode(int8_t num, std::shared_ptr<CiA402Data> data)
  : num_(num),
    data_(data),
    started_(false),
    target_(0.0),
    target_state_(TARGET_REACHED),
    driver_(nullptr)
  {
  }

  virtual void start(lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver> * driver)
  {
    if (!started_)
    {
      driver_ = driver;
      target_state_ = TARGET_REACHED;
      target_ = 0.0;
      started_ = true;
    }
  }

  virtual void stop()
  {
    started_ = false;
    driver_ = nullptr;
    target_state_ = TARGET_REACHED;
    target_ = 0.0;
  }

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
    if (target_state_ == TARGET_ACTIVE)
    {
      return true;
    }
    return false;
  }

  /**
   * @brief Set a new target
   *
   * @param target
   */
  virtual void set_target(double target)
  {
    if (started_ && target_state_ == TARGET_REACHED)
    {
      target_state_ = NEW_TARGET;
      target_ = target;
    }
  }

  double get_target() { return target_; }

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
    bool tr = data_->get_target_reached();
    switch (target_state_)
    {
      case NEW_TARGET:
        if (!tr)
        {
          target_state_ = TARGET_ACTIVE;
        }
        break;
      case TARGET_ACTIVE:
        if (tr)
        {
          target_state_ = TARGET_REACHED;
        }
        break;
      case TARGET_REACHED:
        break;
      default:
        break;
    }
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_MODE_HPP
