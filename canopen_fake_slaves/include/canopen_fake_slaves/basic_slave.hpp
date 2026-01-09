//    Copyright 2022 Christoph Hellmann Santos
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

#ifndef BASIC_SLAVE_HPP
#define BASIC_SLAVE_HPP
#include <lely/coapp/sdo_error.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <fstream>
#include <system_error>
#include <thread>
#include <typeindex>
#include <typeinfo>

#include "canopen_fake_slaves/base_slave.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace lely;
using namespace std::chrono_literals;
namespace ros2_canopen
{
class SimpleSlave : public canopen::BasicSlave
{
public:
  SimpleSlave(
    io::TimerBase & timer, io::CanChannelBase & chan, const std::string & slave_config,
    const std::string & slave_bin, uint8_t node_id)
  : canopen::BasicSlave(timer, chan, slave_config, slave_bin, node_id)
  {
    const auto vendor_id = parse_vendor_id(slave_config);
    if (vendor_id != 0)
    {
      try
      {
        this->OnRead<uint32_t>(
          0x1018, 0x01,
          [vendor_id](uint16_t, uint8_t, uint32_t & value) -> std::error_code
          {
            value = vendor_id;
            return {};
          });
      }
      catch (const lely::canopen::SdoError &)
      {
        // Leave the dictionary untouched if the callback cannot be registered.
      }
    }
  }

  SimpleSlave(
    io::TimerBase & timer, io::CanChannelBase & chan, const std::string & slave_config,
    uint8_t node_id)
  : SimpleSlave(timer, chan, slave_config, "", node_id)
  {
  }

  ~SimpleSlave()
  {
    if (message_thread.joinable())
    {
      message_thread.join();
    }
  }

protected:
  std::thread message_thread;
  static uint32_t parse_vendor_id(const std::string & slave_config)
  {
    std::ifstream stream(slave_config);
    if (!stream.is_open())
    {
      return 0;
    }

    std::string line;
    while (std::getline(stream, line))
    {
      auto pos = line.find("VendorNumber=");
      if (pos == std::string::npos)
      {
        continue;
      }

      auto value_part = line.substr(pos + std::string("VendorNumber=").length());
      try
      {
        return static_cast<uint32_t>(std::stoul(value_part, nullptr, 0));
      }
      catch (const std::exception &)
      {
        return 0;
      }
    }

    return 0;
  }
  /**
   * @brief This function gets an object value through the typed interface.
   * Only supports object types that can fit in a 32-bit container.
   * @param idx The index of the PDO.
   * @param subidx The subindex of the PDO.
   * @return value of object stored in a 32-bit container
   */
  uint32_t GetValue(const uint16_t idx, const uint8_t subidx) const noexcept
  {
    const std::type_index type((*this)[idx][subidx].Type());

    uint32_t value{0};

    if (type == std::type_index(typeid(bool)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<bool>());
    }
    else if (type == std::type_index(typeid(int8_t)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<int8_t>());
    }
    else if (type == std::type_index(typeid(int16_t)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<int16_t>());
    }
    else if (type == std::type_index(typeid(int32_t)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<int32_t>());
    }
    else if (type == std::type_index(typeid(float)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<float>());
    }
    else if (type == std::type_index(typeid(uint8_t)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<uint8_t>());
    }
    else if (type == std::type_index(typeid(uint16_t)))
    {
      value = static_cast<uint32_t>((*this)[idx][subidx].Get<uint16_t>());
    }
    else if (type == std::type_index(typeid(uint32_t)))
    {
      value = (*this)[idx][subidx].Get<uint32_t>();
    }
    else
    {
      value = (*this)[idx][subidx].Get<uint32_t>();
    }

    return value;
  }

  /**
   * @brief This function is called when a value is written to the local object dictionary by an SDO
   * or RPDO. Also copies the RPDO value to TPDO. A function from the class Device
   * @param idx The index of the PDO.
   * @param subidx The subindex of the PDO.
   */
  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    (*this)[0x4001][0] = this->GetValue(idx, subidx);
    this->TpdoEvent(0);

    // Publish periodic message
    if (!message_thread.joinable())
    {
      message_thread = std::thread(std::bind(&SimpleSlave::fake_periodic_messages, this));
    }
  }

  /**
   * @brief This function is attached to a thread and sends periodic messages
   * via 0x4004
   */
  void fake_periodic_messages()
  {
    // If ros is running, send messages
    while (rclcpp::ok())
    {
      uint32_t val = 0x1122;
      (*this)[0x4001][0] = val;  // refresh TPDO-mapped UNSIGNED32 without touching config entries
      this->TpdoEvent(0);
      // 100 ms sleep - 10 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
};

class BasicSlave : public BaseSlave
{
public:
  explicit BasicSlave(const std::string & node_name, bool intra_process_comms = false)
  : BaseSlave(node_name, intra_process_comms)
  {
  }

protected:
  class ActiveCheckTask : public ev::CoTask
  {
  public:
    ActiveCheckTask(io::Context * ctx, ev::Executor * exec, BasicSlave * slave) : CoTask(*exec)
    {
      slave_ = slave;
      exec_ = exec;
      ctx_ = ctx;
    }

  protected:
    BasicSlave * slave_;
    ev::Executor * exec_;
    io::Context * ctx_;
    virtual void operator()() noexcept
    {
      if (slave_->activated.load())
      {
      }
      ctx_->shutdown();
    }
  };

  void run() override
  {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl(can_interface_name_.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    auto sigset_ = lely::io::SignalSet(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset_.insert(SIGHUP);
    sigset_.insert(SIGINT);
    sigset_.insert(SIGTERM);

    sigset_.submit_wait(
      [&](int /*signo*/)
      {
        // If the signal is raised again, terminate immediately.
        sigset_.clear();

        // Perform a clean shutdown.
        ctx.shutdown();
      });

    SimpleSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
    slave.Reset();
    ActiveCheckTask checktask(&ctx, &exec, this);

    // timer.submit_wait()
    RCLCPP_INFO(this->get_logger(), "Created slave for node_id %i.", node_id_);
    loop.run();
    ctx.shutdown();
    RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
    rclcpp::shutdown();
  }
};
}  // namespace ros2_canopen

#endif
