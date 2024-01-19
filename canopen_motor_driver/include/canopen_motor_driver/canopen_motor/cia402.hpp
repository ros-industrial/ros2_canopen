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

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_HPP_

#include <atomic>
#include <condition_variable>
#include <lely/coapp/logical_driver.hpp>
#include <map>
#include <mutex>
#include "cia402_control.hpp"
#include "cia402_data.hpp"
#include "cia402_mode.hpp"
#include "cia402_profile.hpp"
#include "rclcpp/rclcpp.hpp"
namespace ros2_canopen
{

class CiA402 : public lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver>
{
protected:
  const uint16_t control_word_index;
  const uint16_t status_word_index;
  const uint16_t modes_of_operation_index;
  const uint16_t modes_of_operation_display_index;
  const std::chrono::milliseconds boot_check_timeout = std::chrono::milliseconds(1000);
  const rclcpp::Logger logger;

  ros2_canopen::CiA402Data data;
  std::map<int8_t, OperationMode> & modes;

  std::condition_variable cv_switchstate;
  std::mutex mtx_switchstate;
  Status goal_state;
  Status next_state;
  Control command;
  std::atomic_bool switch_state_ongoing;

  std::condition_variable cv_switchmode;
  std::mutex mtx_switchmode;
  int8_t goal_mode;
  std::atomic_bool switch_mode_ongoing;
  OperationMode * current_mode;

  void ReceivedStatusWord(uint16_t status_word) noexcept
  {
    data.set_status(get_status(status_word));
    data.set_target_reached(get_status_bit(status_word, SW_TARGET_REACHED_MASK));
    data.set_internal_limit_active(get_status_bit(status_word, SW_INTERNAL_LIMIT_ACTIVE_MASK));
    data.set_voltage_enabled(get_status_bit(status_word, SW_VOLTAGE_ENABLED_MASK));
  }

public:
  CiA402(
    lely::canopen::BasicDriver & driver, std::map<int8_t, OperationMode> & modes,
    rclcpp::Logger logger = rclcpp::get_logger("canopen_motor_driver"), int num = 1)
  : lely::canopen::BasicLogicalDriver<lely::canopen::BasicDriver>(driver, num, 402),
    control_word_index(ObjectIndex(CIA402_CONTROL_WORD_INDEX)),
    status_word_index(ObjectIndex(CIA402_STATUS_WORD_INDEX)),
    modes_of_operation_index(ObjectIndex(CIA402_MODES_OF_OPERATION_INDEX)),
    modes_of_operation_display_index(CIA402_MODES_OF_OPERATION_DISPLAY_INDEX),
    modes(modes),
    switch_state_ongoing(false),
    logger(logger)
  {
  }

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnSync(uint8_t cnt, const time_point & t) noexcept override;
  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;
  void OnBootCheck() noexcept;
  bool SwitchState(
    Status goal, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool SwitchMode(int8_t goal, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
  bool HasFault();
};
}  // namespace ros2_canopen
#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR__CIA402_HPP_
