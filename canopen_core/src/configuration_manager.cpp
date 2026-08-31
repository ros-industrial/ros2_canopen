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
#include "canopen_core/detail/dcf_preprocessor.hpp"

namespace ros2_canopen
{

void ConfigurationManager::init_config()
{
  // Get the directory containing the bus config file
  std::filesystem::path config_dir = std::filesystem::path(file_).parent_path();
  if (config_dir.empty())
  {
    config_dir = std::filesystem::current_path();
  }

  // Default dcf_path to "." (same directory as bus.yml) for portable install spaces
  std::string dcf_path = ".";
  for (YAML::const_iterator it = root_.begin(); it != root_.end(); it++)
  {
    std::string driver_name = it->first.as<std::string>();
    if (driver_name != "options") continue;
    YAML::Node config_node = it->second;
    if (config_node["dcf_path"])
    {
      dcf_path = config_node["dcf_path"].as<std::string>();
      // Resolve the path (handles relative paths and environment variables)
      dcf_path = detail::resolve_file_path(dcf_path, config_dir);
    }
  }

  // Resolve the dcf_path (handles relative paths and environment variables)
  dcf_path = detail::resolve_file_path(dcf_path, config_dir);

  for (YAML::const_iterator it = root_.begin(); it != root_.end(); it++)
  {
    std::string driver_name = it->first.as<std::string>();
    if (driver_name == "options") continue;
    YAML::Node config_node = it->second;
    if (!config_node["dcf_path"])
    {
      config_node["dcf_path"] = dcf_path;
    }
    else
    {
      // Also resolve per-device dcf_path if specified
      std::string device_dcf_path = config_node["dcf_path"].as<std::string>();
      config_node["dcf_path"] = detail::resolve_file_path(device_dcf_path, config_dir);
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
