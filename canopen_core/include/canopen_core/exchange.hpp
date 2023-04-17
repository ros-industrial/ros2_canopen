//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
#ifndef EXCHANGE_HPP
#define EXCHANGE_HPP

#include <boost/lockfree/queue.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

namespace ros2_canopen
{
enum CODataTypes
{
  CODataUnkown = 0,
  COData8 = 8,
  COData16 = 16,
  COData32 = 32
};

struct COData
{
public:
  uint16_t index_;
  uint8_t subindex_;
  uint32_t data_;
  CODataTypes type_;
};

struct COEmcy
{
public:
  uint16_t eec;
  uint8_t er;
  uint8_t msef[5];
};

/**
 * @brief Thread Safe Queue
 */
template <typename T>
class SafeQueue
{
private:
  std::size_t capacity_;
  std::unique_ptr<boost::lockfree::queue<T>> queue_;

  void create_queue() { queue_ = std::make_shared<boost::lockfree::queue<T>>(capacity_); }

public:
  explicit SafeQueue(std::size_t capacity = 10)
  : capacity_(capacity), queue_(new boost::lockfree::queue<T>(capacity_))
  {
  }

  void push(T value) { queue_->push(std::move(value)); }

  boost::optional<T> try_pop()
  {
    T value;
    if (queue_->pop(value)) return std::optional<T>(std::move(value));
    return boost::none;
  }

  bool try_pop(T & value)
  {
    if (queue_->pop(value)) return true;
    return false;
  }

  boost::optional<T> wait_and_pop()
  {
    T value;
    while (!queue_->pop(value)) boost::this_thread::yield();
    return value;
  }

  void wait_and_pop(T & value)
  {
    while (!queue_->pop(value)) boost::this_thread::yield();
  }

  boost::optional<T> wait_and_pop_for(const std::chrono::milliseconds & timeout)
  {
    T value;
    auto start_time = std::chrono::steady_clock::now();
    while (!queue_->pop(value))
    {
      if (
        timeout != std::chrono::milliseconds::zero() &&
        std::chrono::steady_clock::now() - start_time >= timeout)
        return boost::none;
      boost::this_thread::yield();
    }
    return value;
  }

  bool wait_and_pop_for(const std::chrono::milliseconds & timeout, T & value)
  {
    auto start_time = std::chrono::steady_clock::now();
    while (!queue_->pop(value))
    {
      if (
        timeout != std::chrono::milliseconds::zero() &&
        std::chrono::steady_clock::now() - start_time >= timeout)
        return false;
      boost::this_thread::yield();
    }
    return true;
  }

  bool empty() const { return queue_->empty(); }
};
}  // namespace ros2_canopen

#endif  // EXCHANGE_HPP