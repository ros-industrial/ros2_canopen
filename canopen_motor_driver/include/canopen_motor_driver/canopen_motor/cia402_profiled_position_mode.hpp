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
#include "cia402_data.hpp"
#include "lely/coapp/logical_driver.hpp"

class Pr
{
protected:
  const int8_t num_;

public:
  OperationMode(int8_t num, ) : num_(num) {}
  virtual bool on_init() {}
  virtual void on_boot() {}
  virtual bool on_boot_check(uint32_t supported_modes)
  {
    return (supported_modes >> (num_ - 1)) & 0x1;
  }
  virtual void on_set_target(double target) {}
  virtual void on_rpdo() {}
  virtual void on_sync() {}
};

#endif  // CANOPEN_MOTOR_DRIVER__CANOPEN_MOTOR_CIA402_MODE_HPP_
