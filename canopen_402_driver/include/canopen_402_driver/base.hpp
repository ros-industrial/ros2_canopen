//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2014-2022 Authors of ros_canopen
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
#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H
#include <string>

#include "lely/coapp/driver.hpp"
#include "lely/coapp/master.hpp"

namespace ros2_canopen
{

/**
 * @brief Motor Base Class
 *
 */
class MotorBase
{
protected:
  MotorBase() {}

public:
  enum OperationMode
  {
    No_Mode = 0,
    Profiled_Position = 1,
    Velocity = 2,
    Profiled_Velocity = 3,
    Profiled_Torque = 4,
    Reserved = 5,
    Homing = 6,
    Interpolated_Position = 7,
    Cyclic_Synchronous_Position = 8,
    Cyclic_Synchronous_Velocity = 9,
    Cyclic_Synchronous_Torque = 10,
  };

  /**
   * @brief Set target
   *
   * @param [in] val      Target value
   * @return true
   * @return false
   */
  virtual bool setTarget(double val) = 0;

  /**
   * @brief Enter Operation Mode
   *
   * @param [in] mode     Target Mode
   * @return true
   * @return false
   */
  virtual bool enterModeAndWait(uint16_t mode) = 0;

  /**
   * @brief Check if Operation Mode is supported
   *
   * @param [in] mode     Operation Mode to be checked
   * @return true
   * @return false
   */
  virtual bool isModeSupported(uint16_t mode) = 0;

  /**
   * @brief Get current Mode
   *
   * @return uint16_t
   */
  virtual uint16_t getMode() = 0;

  /**
   * @brief Register default Operation Modes
   *
   */
  virtual void registerDefaultModes() {}

  typedef std::shared_ptr<MotorBase> MotorBaseSharedPtr;
};
typedef MotorBase::MotorBaseSharedPtr MotorBaseSharedPtr;

}  // namespace ros2_canopen

#endif
