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

#include "canopen_core/configuration_manager.hpp"
#include <filesystem>
#include <stdexcept>

namespace ros2_canopen
{

void ConfigurationManager::init_config()
{
  // Get the directory of the bus config file for resolving relative paths
  std::filesystem::path bus_config_dir = std::filesystem::path(file_).parent_path();

  std::filesystem::path dcf_path = "";
  for (YAML::const_iterator it = root_.begin(); it != root_.end(); it++)
  {
    std::string driver_name = it->first.as<std::string>();
    if (driver_name != "options") continue;
    YAML::Node config_node = it->second;
    if (config_node["dcf_path"])
    {
      dcf_path = config_node["dcf_path"].as<std::string>();
    }
  }

  for (YAML::const_iterator it = root_.begin(); it != root_.end(); it++)
  {
    std::string driver_name = it->first.as<std::string>();
    if (driver_name == "options") continue;
    YAML::Node config_node = it->second;

    if (config_node["dcf_path"])
    {
      dcf_path = config_node["dcf_path"].as<std::string>();
    }
    // Resolve relative paths in individual device configs
    if (dcf_path.is_relative())
    {
      dcf_path = bus_config_dir / dcf_path;
      dcf_path = std::filesystem::absolute(dcf_path);
      config_node["dcf_path"] = dcf_path.string();

      RCLCPP_INFO(
        rclcpp::get_logger("ConfigurationManager"),
        "Resolved relative dcf_path for device '%s' to absolute path '%s' (relative to bus config "
        "'%s')",
        driver_name.c_str(), dcf_path.string().c_str(), file_.c_str());
    }
    devices_.insert({driver_name, config_node});
  }
}

uint32_t ConfigurationManager::get_all_devices(std::vector<std::string> & devices)
{
  uint32_t count = 0;
  for (auto it = devices_.begin(); it != devices_.end(); it++)
  {
    devices.emplace_back(it->first);
    count++;
  }
  return count;
}

}  // namespace ros2_canopen
