// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_

#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>

#include "canopen_core/device_container.hpp"
#include "canopen_proxy_driver/proxy_driver.hpp"
#include "canopen_ros2_control/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace canopen_ros2_control
{
template<typename TargetType, typename SourceType>
auto makeMemcpyCaster(const SourceType& source)
{
    return [&source]() -> TargetType {
        TargetType target;
        std::memcpy(&target, &source, sizeof(TargetType));
        return target;
    };
}

const std::vector<std::string> SUPPORTED_TYPES = {
  "bool",
  "int8_t",
  "uint8_t",
  "int16_t",
  "uint16_t",
  "int32_t",
  "uint32_t"
};

// needed auxiliary struct for ros2 control double registration
struct Ros2ControlCOData
{
  // TODO(Dr.Denis): rename original data to canopen data
  // Soon we can drop this as ros2_control support variants - we have to add support for all this types
  ros2_canopen::COData original_data;

  double index;     // cast to uint16_t
  double subindex;  // cast to uint8_t
  double data;      // cast to uint32_t

  std::string co_type = "int32_t";  // use int32_t as default

  void set_co_data_type(const std::string & type)
  {
    if (type.empty() || std::find(SUPPORTED_TYPES.begin(), SUPPORTED_TYPES.end(), type) != SUPPORTED_TYPES.end())
    {
      co_type = type;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Type '%s' empty or not yet supported. Using 'int32_t' as default to cast directly to double. This might cause errornous data! Please contribute or by maintainers a cookie and hope they implement it for you!", type.c_str());
      co_type = "int32_t";
    }
  }
};



struct RORos2ControlCOData : public Ros2ControlCOData
{
  void set_data(ros2_canopen::COData d)
  {
    original_data = d;
  }

  void prepare_data()
  {
    index = static_cast<double>(original_data.index_);
    subindex = static_cast<double>(original_data.subindex_);

    if (co_type == "bool")
    {
      bool bool_data;
      std::memcpy(&bool_data, &original_data.data_, sizeof(bool));
      data = static_cast<double>(bool_data);
    }
    else if (co_type == "int8_t")
    {
      int8_t int8_data;
      std::memcpy(&int8_data, &original_data.data_, sizeof(int8_t));
      data = static_cast<double>(int8_data);
    }
    else if (co_type == "uint8_t")
    {
      uint8_t uint8_data;
      std::memcpy(&uint8_data, &original_data.data_, sizeof(uint8_t));
      data = static_cast<double>(uint8_data);
    }
    else if (co_type == "int16_t")
    {
      int16_t int16_data;
      std::memcpy(&int16_data, &original_data.data_, sizeof(int16_t));
      data = static_cast<double>(int16_data);
    }
    else if (co_type == "uint16_t")
    {
      uint16_t uint16_data;
      std::memcpy(&uint16_data, &original_data.data_, sizeof(uint16_t));
      data = static_cast<double>(uint16_data);
    }
    else if (co_type == "int32_t")
    {
      int32_t int32_data;
      std::memcpy(&int32_data, &original_data.data_, sizeof(int32_t));
      data = static_cast<double>(int32_data);
    }
    else if (co_type == "uint32_t")
    {
      uint32_t uint32_data;
      std::memcpy(&uint32_data, &original_data.data_, sizeof(uint32_t));
      data = static_cast<double>(uint32_data);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Type '%s' not yet supported. Trying to cast directly to double. This might cause errornous data! Please contribute or by maintainers a cookie and hope they implement it for you!", co_type.c_str());
      data = static_cast<double>(original_data.data_);
    }
  }
};

struct WORos2ControlCoData : public Ros2ControlCOData
{
  WORos2ControlCoData() : one_shot(std::numeric_limits<double>::quiet_NaN()) {}

  // needed internally for write-only data
  double one_shot;

  bool write_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(one_shot);
    // reset the existing active command if one exists
    one_shot = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }

  void prepare_data()
  {
    original_data.index_ = static_cast<uint16_t>(index);
    original_data.subindex_ = static_cast<uint8_t>(subindex);

    if (co_type == "bool")
    {
      bool bool_data = static_cast<bool>(data);
      std::memcpy(&original_data.data_, &bool_data, sizeof(bool));
    }
    else if (co_type == "int8_t")
    {
      int8_t int8_data = static_cast<int8_t>(data);
      std::memcpy(&original_data.data_, &int8_data, sizeof(int8_t));
    }
    else if (co_type == "uint8_t")
    {
      uint8_t uint8_data = static_cast<uint8_t>(data);
      std::memcpy(&original_data.data_, &uint8_data, sizeof(uint8_t));
    }
    else if (co_type == "int16_t")
    {
      int16_t int16_data = static_cast<int16_t>(data);
      std::memcpy(&original_data.data_, &int16_data, sizeof(int16_t));
    }
    else if (co_type == "uint16_t")
    {
      uint16_t uint16_data = static_cast<uint16_t>(data);
      std::memcpy(&original_data.data_, &uint16_data, sizeof(uint16_t));
    }
    else if (co_type == "int32_t")
    {
      int32_t int32_data = static_cast<int32_t>(data);
      std::memcpy(&original_data.data_, &int32_data, sizeof(int32_t));
    }
    else if (co_type == "uint32_t")
    {
      uint32_t uint32_data = static_cast<uint32_t>(data);
      std::memcpy(&original_data.data_, &uint32_data, sizeof(uint32_t));
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Type '%s' not yet supported. Trying to cast directly to uint32_t. This might cause errornous data! Please contribute or by maintainers a cookie and hope they implement it for you!", co_type.c_str());
      original_data.data_ = static_cast<uint32_t>(data);
    }
  }
};

struct Ros2ControlEmcyData
{
  Ros2ControlEmcyData()
  : error_code(0),
    error_register(0),
    manufacturer_error_code1(0),
    manufacturer_error_code2(0),
    manufacturer_error_code3(0),
    manufacturer_error_code4(0),
    manufacturer_error_code5(0)
  {
  }

  void set_emcy(ros2_canopen::COEmcy e)
  {
    original_emcy = e;

    uint16_t eec_data;
    std::memcpy(&eec_data, &e.eec, sizeof(uint16_t));
    error_code = static_cast<double>(eec_data);

    uint8_t er_data;
    std::memcpy(&er_data, &e.er, sizeof(uint8_t));
    error_register = static_cast<double>(er_data);

    uint16_t msef_data;
    std::memcpy(&msef_data, &e.msef[0], sizeof(uint16_t));
    manufacturer_error_code1 = static_cast<double>(msef_data);
    std::memcpy(&msef_data, &e.msef[1], sizeof(uint16_t));
    manufacturer_error_code2 = static_cast<double>(msef_data);
    std::memcpy(&msef_data, &e.msef[2], sizeof(uint16_t));
    manufacturer_error_code3 = static_cast<double>(msef_data);
    std::memcpy(&msef_data, &e.msef[3], sizeof(uint16_t));
    manufacturer_error_code4 = static_cast<double>(msef_data);
    std::memcpy(&msef_data, &e.msef[4], sizeof(uint16_t));
    manufacturer_error_code5 = static_cast<double>(msef_data);
  }

  double error_code;             // read-only
  double error_register;         // read-only
  double manufacturer_error_code1;  // read-only
  double manufacturer_error_code2;  // read-only
  double manufacturer_error_code3;  // read-only
  double manufacturer_error_code4;  // read-only
  double manufacturer_error_code5;  // read-only

  ros2_canopen::COEmcy original_emcy;  // read-only
};

struct Ros2ControlNmtState
{
  Ros2ControlNmtState()
  : reset_ons(std::numeric_limits<double>::quiet_NaN()),
    reset_fbk(std::numeric_limits<double>::quiet_NaN()),
    start_ons(std::numeric_limits<double>::quiet_NaN()),
    start_fbk(std::numeric_limits<double>::quiet_NaN())
  {
  }

  void set_state(canopen::NmtState s)
  {
    original_state = s;
    state = static_cast<double>(s);
  }

  bool reset_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(reset_ons);
    // reset the existing active command if one exists
    reset_ons = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }

  bool start_command()
  {
    bool ret_val;
    // store ret value
    ret_val = !std::isnan(start_ons);
    // reset the existing active command if one exists
    start_ons = std::numeric_limits<double>::quiet_NaN();
    return ret_val;
  }
  canopen::NmtState original_state;

  double state;  // read-only

  // basic commands
  double reset_ons;  // write-only
  double reset_fbk;
  double start_ons;  // write-only
  double start_fbk;
};

struct pair_hash
{
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> & pair) const
  {
    auto h1 = std::hash<T1>{}(pair.first);
    auto h2 = std::hash<T2>{}(pair.second);

    return h1 ^ h2;
  }
};

struct CanopenNodeData
{
  Ros2ControlNmtState nmt_state;  // read-write
  RORos2ControlCOData rpdo_data;  // read-only
  WORos2ControlCoData tpdo_data;  // write-only

  Ros2ControlEmcyData emcy_data;  // read-only

  WORos2ControlCoData rsdo;  // write-only
  WORos2ControlCoData wsdo;  // write-only

  using PDO_INDICES = std::pair<uint16_t, uint8_t>;  // Index, Subindex
  std::unordered_map<PDO_INDICES, double, pair_hash> rpdo_data_map;  // kept for backward compoatibility - should b removed
  std::unordered_map<PDO_INDICES, uint32_t, pair_hash> rpdo_raw_data_map;

  // Push data to the queue - FIFO
  void set_rpdo_data(ros2_canopen::COData d)
  {
    rpdo_data.set_data(d);
    rpdo_data.prepare_data();

    PDO_INDICES index_pair(d.index_, d.subindex_);

    // check if the index pair is already in the map
    if (rpdo_raw_data_map.find(index_pair) != rpdo_raw_data_map.end())
    {
      // if it is, update the value
      rpdo_raw_data_map.at(index_pair) = d.data_;
    }
    else
    {
      // if it is not, add it to the map
      rpdo_raw_data_map.emplace(index_pair, d.data_);
    }
    // TODO(Dr.Denis): kept for backward compatibility - should be removed
    // check if the index pair is already in the map
    if (rpdo_data_map.find(index_pair) != rpdo_data_map.end())
    {
      // if it is, update the value
      rpdo_data_map.at(index_pair) = rpdo_data.data;
    }
    else
    {
      // if it is not, add it to the map
      rpdo_data_map.emplace(index_pair, rpdo_data.data);
    }
  }

  uint32_t get_rpdo_raw_data(uint16_t index, uint8_t subindex)
  {
    PDO_INDICES index_pair(index, subindex);
    if (rpdo_raw_data_map.find(index_pair) != rpdo_raw_data_map.end())
    {
      return rpdo_raw_data_map.at(index_pair);
    }
    else
    {
      // Log error
      // RCLCPP_WARN(kLogger, "The index pair (%u, %u) is not in the map", index, subindex);
      return std::numeric_limits<uint32_t>::quiet_NaN();
    }
  }

  // Pop data from the queue
  double get_rpdo_data(uint16_t index, uint8_t subindex)
  {
    PDO_INDICES index_pair(index, subindex);
    if (rpdo_data_map.find(index_pair) != rpdo_data_map.end())
    {
      return rpdo_data_map[index_pair];
    }
    else
    {
      // // Log error
      // RCLCPP_WARN(kLogger, "The index pair (%u, %u) is not in the map", index, subindex);
      return std::numeric_limits<double>::quiet_NaN();
    }
  }
};

class CanopenSystem : public hardware_interface::SystemInterface
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  CanopenSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~CanopenSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ros2_canopen::DeviceContainer> device_container_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  // can stuff
  std::map<uint16_t, CanopenNodeData> canopen_data_;
  // threads
  std::unique_ptr<std::thread> spin_thread_;
  std::unique_ptr<std::thread> init_thread_;

  void spin();
  void clean();

private:
  void initDeviceContainer();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CANOPEN_SYSTEM_HPP_
