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

#include "canopen_motor_driver/canopen_motor/cia402.hpp"
#include "lely/ev/co_task.hpp"

using namespace ros2_canopen;
using namespace std::chrono_literals;

void CiA402::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept {}

void CiA402::OnSync(uint8_t cnt, const time_point & t) noexcept
{
  data.set_status_word(driver.rpdo_mapped[status_word_index][0x0]);
  data.set_mode_of_operation_display(driver.rpdo_mapped[modes_of_operation_display_index][0x0]);

  if (switch_state_ongoing.load())
  {
    std::lock_guard<std::mutex> lk(mtx_switchstate);
    if (data.get_status() == next_state)
    {
      cv_switchstate.notify_one();
    }
  }

  if (switch_mode_ongoing.load())
  {
    std::lock_guard<std::mutex> lk(mtx_switchmode);
    if (data.get_mode_of_operation_display() == goal_mode)
    {
      cv_switchmode.notify_one();
    }
  }

  if (!switch_mode_ongoing.load() && !switch_state_ongoing.load())
  {
    if (current_mode != nullptr)
    {
      current_mode->on_sync();
    }
  }

  driver.tpdo_mapped[control_word_index][0x0] = data.get_control_word();
  driver.tpdo_mapped[modes_of_operation_index][0x0] = data.get_mode_of_operation();
}

void CiA402::OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept
{
  SubmitRead<uint32_t>(
    0x6502, 0x0,
    [this](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, uint32_t value)
    { data.set_supported_modes(value); });
  for (auto & mode : modes)
  {
    mode.second.on_boot((*this));
  }
  // lely::io::TimerBase::
  SubmitWait(boot_check_timeout, [this](std::error_code ec) { OnBootCheck(); });
}

void CiA402::OnBootCheck() noexcept
{
  for (auto & mode : modes)
  {
    mode.second.on_boot_check();
  }
}

bool CiA402::SwitchState(
  Status goal, std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
{
  if (switch_state_ongoing.load())
  {
    RCLCPP_WARN(logger, "DENIED Switch State: Switch state already ongoing.");
    return false;
  }
  switch_state_ongoing.store(true);
  if (goal == data.get_status())
  {
    RCLCPP_DEBUG(logger, "SUCCESS Switch State: Already in goal state.");
    switch_state_ongoing.store(false);
    return true;
  }

  goal_state = goal;
  while (data.get_status() != goal_state)
  {
    next_state = GetNextStatus(data.get_status(), goal_state);
    if (next_state == Status::UNKNOWN)
    {
      RCLCPP_ERROR(
        logger, "DENIED Switch State: Unknown next state for transition '%d' to '%d'.",
        data.get_status(), goal_state);
      switch_state_ongoing.store(false);
      return false;
    }
    command = GetNextCommand(data.get_status(), next_state);
    if (command == Control::UNKNOWN)
    {
      RCLCPP_ERROR(
        logger, "DENIED Switch State: Unknown command for transition '%d' to '%d'.",
        data.get_status(), next_state);
      switch_state_ongoing.store(false);
      return false;
    }
    data.set_command(command);
    std::unique_lock<std::mutex> lk(mtx_switchstate);
    auto cv_status = cv_switchstate.wait_for(lk, timeout);

    if (cv_status == std::cv_status::timeout)
    {
      RCLCPP_ERROR(
        logger, "DENIED Switch State: Timeout while waiting for state transition '%d' to '%d'.",
        data.get_status(), next_state);
      switch_state_ongoing.store(false);
      return false;
    }

    if (data.get_status() != next_state)
    {
      RCLCPP_ERROR(
        logger, "DENIED Switch State: Unexpected state transition '%d' to '%d'.", data.get_status(),
        next_state);
      switch_state_ongoing.store(false);
      return false;
    }
  }
  RCLCPP_DEBUG(logger, "SUCCESS Switch State: Reached goal state '%d'.", goal_state);
  switch_state_ongoing.store(false);
  return true;
}

bool CiA402::SwitchMode(int8_t goal, std::chrono::milliseconds timeout)
{
  if (switch_mode_ongoing.load())
  {
    RCLCPP_WARN(logger, "DENIED Switch Mode: Switch mode already ongoing.");
    return false;
  }
  switch_mode_ongoing.store(true);
  if (goal == data.get_mode_of_operation_display())
  {
    RCLCPP_DEBUG(logger, "SUCCESS Switch Mode: Already in goal mode.");
    switch_mode_ongoing.store(false);
    return true;
  }

  goal_mode = goal;
  auto mode = modes.find(goal_mode);
  if (mode == modes.end())
  {
    RCLCPP_ERROR(logger, "DENIED Switch Mode: Unknown mode '%d'.", goal_mode);
    switch_mode_ongoing.store(false);
    return false;
  }

  if (current_mode != nullptr)
  {
    current_mode->stop();
    current_mode = nullptr;
  }

  mode->second.start(this);
  data.set_mode_of_operation(goal_mode);
  std::unique_lock<std::mutex> lk(mtx_switchmode);
  auto cv_status = cv_switchmode.wait_for(lk, timeout);

  if (cv_status == std::cv_status::timeout)
  {
    RCLCPP_ERROR(
      logger, "DENIED Switch Mode: Timeout while waiting for mode transition '%d'.", goal);
    switch_mode_ongoing.store(false);
    return false;
  }

  if (data.get_mode_of_operation_display() != goal_mode)
  {
    RCLCPP_ERROR(
      logger, "DENIED Switch Mode: Unexpected mode transition '%d' to '%d'.",
      data.get_mode_of_operation_display(), goal_mode);
    switch_mode_ongoing.store(false);
    return false;
  }
  RCLCPP_DEBUG(logger, "SUCCESS Switch Mode: Reached goal mode '%d'.", goal_mode);
  current_mode = &(mode->second);
  switch_mode_ongoing.store(false);
  return true;
}

bool CiA402::HasFault() { return data.get_status_word_bit(SW_FAULT_MASK); }
