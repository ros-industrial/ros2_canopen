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

#ifndef HOMING_MODE_HPP
#define HOMING_MODE_HPP
#include "base.hpp"
#include "mode.hpp"

namespace ros2_canopen
{
class HomingMode : public Mode
{
protected:
  enum SW_bits
  {
    SW_Attained = State402::SW_Operation_mode_specific0,
    SW_Error = State402::SW_Operation_mode_specific1,
  };
  enum CW_bits
  {
    CW_StartHoming = Command402::CW_Operation_mode_specific0,
  };

public:
  HomingMode() : Mode(MotorBase::Homing) {}
  virtual bool executeHoming() = 0;
};
}  // namespace ros2_canopen

#endif  // HOMING_MODE_HPP
