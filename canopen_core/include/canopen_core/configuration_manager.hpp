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

#ifndef CONFIGURATION_MANAGER_HPP
#define CONFIGURATION_MANAGER_HPP

#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <optional>
#include "yaml-cpp/yaml.h"

namespace ros2_canopen
{
    class ConfigurationManager
    {
    private:
        std::string file_;
        YAML::Node root_;

        std::map<std::string, YAML::Node> devices_;

    public:
        ConfigurationManager(std::string &file) : file_(file)
        {
            root_ = YAML::LoadFile(file_.c_str());
        }

        template <typename T>
        std::optional<T> get_entry(std::string device_name, std::string entry_name)
        {
            try
            {
                auto config = devices_.at(device_name);
                return std::optional<T>(config[entry_name.c_str()].as<T>());
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }

            return std::nullopt;
        }

        void init_config();
        uint32_t get_all_devices(std::vector<std::string> &devices);
    };
}

#endif