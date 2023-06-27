// Copyright 2023 ROS-Industrial
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

#ifndef DIAGNOSTICS_COLLECTOR_HPP_
#define DIAGNOSTICS_COLLECTOR_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"

namespace ros2_canopen
{

/**
 * @brief A class to collect diagnostic information
 *
 */
class DiagnosticsCollector
{
public:
  DiagnosticsCollector() : level_(diagnostic_msgs::msg::DiagnosticStatus::OK) {}

  /**
   * @brief Get the Level
   * Returns the current level (OK, WARN, ERROR or STALE) of the diagnostic
   * @return unsigned char
   */
  unsigned char getLevel() const
  {
    return static_cast<unsigned char>(level_.load(std::memory_order_relaxed));
  }

  /**
   * @brief Get the Message object
   * Returns the current message of the diagnostic
   * @return std::string
   */
  std::string getMessage() const { return message_; }

  /**
   * @brief Get the Value object
   * Returns the current different device state values of the diagnostic.
   * @param key Search device state by key. Eg. "DEVICE", "NMT", "EMCY", "cia402_state",
   * "cia402_mode" etc.
   * @return std::string
   */
  std::string getValue(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);
    if (it != values_.end())
      return it->second;
    else
      return "";  // Return an empty string if key not found
  }

  /**
   * @brief Store current device summary
   *
   * @param lvl Operation level (OK, WARN, ERROR or STALE)
   * @param message Device summary message
   */
  void summary(unsigned char lvl, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->setLevel(lvl);
    this->setMessage(message);
  }

  /**
   * @brief Store current device summary
   *
   * @param lvl Operation level (OK, WARN, ERROR or STALE)
   * @param format Device summary message format
   * @param ...
   */
  void summaryf(unsigned char lvl, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    summary(lvl, std::string(buffer));
  }

  /**
   * @brief Add a device state value
   *
   * @param key Device state key. Eg. "DEVICE", "NMT", "EMCY", "cia402_state", "cia402_mode" etc.
   * @param value Current device state value
   */
  void add(const std::string & key, const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->setValue(key, value);
  }

  /**
   * @brief Add a device state value
   *
   * @param key Device state key. Eg. "DEVICE", "NMT", "EMCY", "cia402_state", "cia402_mode" etc.
   * @param format Current device state value format
   * @param ...
   */
  void addf(const std::string & key, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    add(key, std::string(buffer));
  }

  /**
   * @brief Update all diagnostic information
   *
   * @param lvl Operation level (OK, WARN, ERROR or STALE)
   * @param message Device summary message
   * @param key Device state key. Eg. "DEVICE", "NMT", "EMCY", "cia402_state", "cia402_mode" etc.
   * @param value Current device state value
   */
  void updateAll(
    unsigned char lvl, const std::string & message, const std::string & key,
    const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->setLevel(lvl);
    this->setMessage(message);
    this->setValue(key, value);
  }

private:
  std::atomic<unsigned char> level_;
  std::string message_;
  std::unordered_map<std::string, std::string> values_;
  mutable std::mutex mutex_;

  void setLevel(unsigned char lvl)
  {
    level_.store(static_cast<unsigned char>(lvl), std::memory_order_relaxed);
  }

  void setMessage(const std::string & message) { message_ = message; }

  void setValue(const std::string & key, const std::string & value) { values_[key] = value; }

  // std::unordered_map<std::string, std::string> getValues() const
  // {
  //     std::lock_guard<std::mutex> lock(mutex_);
  //     return values_;
  // }
};
}  // namespace ros2_canopen

#endif  // DIAGNOSTICS_COLLECTOR_HPP_
