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

#ifndef MODE_TARGET_HELPER_HPP
#define MODE_TARGET_HELPER_HPP

#include <atomic>
#include <boost/numeric/conversion/cast.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "canopen_402_driver/mode.hpp"

namespace ros2_canopen
{

template <typename T>
class ModeTargetHelper : public Mode
{
  T target_;
  std::atomic<bool> has_target_;

public:
  ModeTargetHelper(uint16_t mode) : Mode(mode) {}
  bool hasTarget() { return has_target_; }
  T getTarget() { return target_; }
  virtual bool setTarget(const double & val)
  {
    if (std::isnan(val))
    {
      // std::cout << "canopen_402 target command is not a number" << std::endl;
      RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_target"), "Target command is not a number");
      return false;
    }

    using boost::numeric_cast;
    using boost::numeric::negative_overflow;
    using boost::numeric::positive_overflow;

    try
    {
      target_ = numeric_cast<T>(val);
    }
    catch (negative_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to min limit" << std::endl;
      target_ = std::numeric_limits<T>::min();
    }
    catch (positive_overflow &)
    {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to max limit" << std::endl;
      target_ = std::numeric_limits<T>::max();
    }
    catch (...)
    {
      std::cout << "canopen_402 Was not able to cast command " << val << std::endl;
      return false;
    }

    has_target_ = true;
    return true;
  }
  virtual bool start()
  {
    has_target_ = false;
    return true;
  }
};
}  // namespace ros2_canopen

#endif  // MODE_TARGET_HELPER_HPP
