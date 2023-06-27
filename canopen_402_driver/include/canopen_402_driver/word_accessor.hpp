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

#ifndef WORD_ACCESSOR_HPP
#define WORD_ACCESSOR_HPP

#include <cstdint>

namespace ros2_canopen
{
template <uint16_t MASK>
class WordAccessor
{
  uint16_t & word_;

public:
  WordAccessor(uint16_t & word) : word_(word) {}
  bool set(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ |= val;
    return val;
  }
  bool reset(uint8_t bit)
  {
    uint16_t val = MASK & (1 << bit);
    word_ &= ~val;
    return val;
  }
  bool get(uint8_t bit) const { return word_ & (1 << bit); }
  uint16_t get() const { return word_ & MASK; }
  WordAccessor & operator=(const uint16_t & val)
  {
    word_ = (word_ & ~MASK) | (val & MASK);
    return *this;
  }
};
}  // namespace ros2_canopen

#endif  // WORD_ACCESSOR_HPP
