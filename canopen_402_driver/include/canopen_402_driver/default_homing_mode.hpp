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

#ifndef DEFAULT_HOMING_MODE_HPP
#define DEFAULT_HOMING_MODE_HPP
#include <mutex>
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "homing_mode.hpp"

namespace ros2_canopen
{

class DefaultHomingMode : public HomingMode
{
  const uint16_t index = 0x6098;
  std::shared_ptr<LelyDriverBridge> driver;

  std::atomic<bool> execute_;

  std::mutex mutex_;
  std::condition_variable cond_;
  uint16_t status_;

  enum SW_masks
  {
    MASK_Reached = (1 << State402::SW_Target_reached),
    MASK_Attained = (1 << SW_Attained),
    MASK_Error = (1 << SW_Error),
  };
  bool error(const std::string & msg)
  {
    execute_ = false;
    std::cout << msg << std::endl;
    return false;
  }

public:
  DefaultHomingMode(std::shared_ptr<LelyDriverBridge> driver) { this->driver = driver; }
  virtual bool start();
  virtual bool read(const uint16_t & sw);
  virtual bool write(OpModeAccesser & cw);

  virtual bool executeHoming();
};
}  // namespace ros2_canopen
#endif  // DEFAULT_HOMING_MODE_HPP
