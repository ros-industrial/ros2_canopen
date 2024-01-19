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

#ifndef CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_SWITCH_STATE_TASK_HPP_
#define CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_SWITCH_STATE_TASK_HPP_
#include <lely/ev/co_task.hpp>
#include "cia402_data.hpp"

namespace ros2_canopen
{
class SwitchStateTask : public lely::ev::CoTask
{
private:
  ros2_canopen::CiA402Data * data_;
  Status goal_;
  Status next_goal_;
  Control command_;
  bool result_;

public:
  SwitchStateTask() : lely::ev::CoTask() {}
  virtual ~SwitchStateTask() = default;
  virtual void operator()() noexcept override
  {
    co_reenter(*this)
    {
      next_goal_ = GetNextStatus(data_->get_status(), goal_);
      if (next_goal_ == Status::UNKNOWN)
      {
        result_ = false;
        return;
      }
      command_ = GetNextCommand(data_->get_status(), next_goal_);
      if (command == Control::UNKNOWN)
      {
        result_ = false;
        return;
      }
      data_->set_command(command);
      while (data_->get_status() != next_goal_)
      {
        co_await data_->wait_for_status(status);
      }
    }
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_SWITCH_STATE_TASK_HPP_
