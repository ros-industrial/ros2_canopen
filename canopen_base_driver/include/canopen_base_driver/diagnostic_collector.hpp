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
class DiagnosticsCollector
{
public:
  DiagnosticsCollector() : level_(diagnostic_msgs::msg::DiagnosticStatus::OK) {}

  unsigned char getLevel() const
  {
    return static_cast<unsigned char>(level_.load(std::memory_order_relaxed));
  }

  std::string getMessage() const { return message_; }

  std::string getValue(const std::string & key) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = values_.find(key);
    if (it != values_.end())
      return it->second;
    else
      return "";  // Return an empty string if key not found
  }

  void summary(unsigned char lvl, const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->setLevel(lvl);
    this->setMessage(message);
  }

  void summayf(unsigned char lvl, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    summary(lvl, std::string(buffer));
  }

  void add(const std::string & key, const std::string & value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->setValue(key, value);
  }

  void addf(const std::string & key, const char * format, ...)
  {
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    add(key, std::string(buffer));
  }

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
