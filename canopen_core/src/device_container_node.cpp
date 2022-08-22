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

#include "canopen_core/device_container_node.hpp"

using namespace ros2_canopen;

void DeviceContainerNode::set_executor(const std::weak_ptr<rclcpp::Executor> executor)
{
    executor_ = executor;
}

void DeviceContainerNode::add_node_to_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name)
{
    if (auto exec = executor_.lock())
    {
        exec->add_node(node_wrappers_[node_id].get_node_base_interface(), true);

        auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());

        active_drivers_.insert({node_name, std::make_pair(node_id, driver_name)});

        RCLCPP_INFO(this->get_logger(), "Added node of type %s with name \"%s\" for node_id %hhu to executor.", driver_name.c_str(), node_name.c_str(), node_id);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to add %s of type %s to executor", node_name.c_str(), driver_name.c_str());
    }
}

void DeviceContainerNode::add_driver_to_master(std::string driver_name, uint8_t node_id)
{
    RCLCPP_INFO(this->get_logger(), "Adding %s for node id %u to master loop.", driver_name.c_str(), node_id);
    auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());
    can_master_->add_driver(node_instance, node_id);
}

void DeviceContainerNode::remove_node_from_executor(const std::string &driver_name, const uint8_t node_id, const std::string &node_name)
{
    RCLCPP_INFO(this->get_logger(), "Removing %s", driver_name.c_str());
    if (auto exec = executor_.lock())
    {
        exec->remove_node(node_wrappers_[node_id].get_node_base_interface());

        auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());

        RCLCPP_INFO(this->get_logger(), "Removed %s of type %s from executor", node_name.c_str(), driver_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to remove %s of type %s from executor", node_name.c_str(), driver_name.c_str());
    }
}

void DeviceContainerNode::remove_driver_from_master(uint8_t node_id)
{
    auto node_instance = std::static_pointer_cast<ros2_canopen::DriverInterface>(node_wrappers_[node_id].get_node_instance());
    can_master_->remove_driver(node_instance, node_id);
}

bool DeviceContainerNode::load_component(std::string &package_name, std::string &driver_name, uint8_t node_id, std::string &node_name)
{
    ComponentResource component;
    std::vector<ComponentResource> components = this->get_component_resources(package_name);
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        if (it->first.compare(driver_name) == 0)
        {
            auto factory_node = this->create_component_factory(*it);
            rclcpp::NodeOptions opts;
            opts.use_global_arguments(false);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            opts.arguments(remap_rules);

            try
            {
                node_wrappers_[node_id] = factory_node->create_node_instance(opts);
            }
            catch (const std::exception &ex)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException(
                    "Component constructor threw an exception: " + std::string(ex.what()));
            }
            catch (...)
            {
                // In the case that the component constructor throws an exception,
                // rethrow into the following catch block.
                throw rclcpp_components::ComponentManagerException("Component constructor threw an exception");
            }

            return true;
        }
    }
    return false;
}

std::map<uint32_t, std::string> DeviceContainerNode::list_components()
{
    std::map<uint32_t, std::string> components;
    for (auto &wrapper : node_wrappers_)
    {
        components[wrapper.first] =
            wrapper.second.get_node_base_interface()->get_fully_qualified_name();
    }
    return components;
}

bool DeviceContainerNode::add_master(uint8_t node_id)
{
    RCLCPP_INFO(this->get_logger(), "Adding master with node id %u", node_id);

    can_master_ = std::static_pointer_cast<ros2_canopen::MasterInterface>(node_wrappers_[node_id].get_node_instance());
    can_master_->init(dcf_txt_, dcf_bin_, can_interface_name_, node_id, config_);
    return true;
}

bool DeviceContainerNode::init_devices_from_config()
{
    std::vector<std::string> devices;
    uint32_t count = this->config_->get_all_devices(devices);
    RCLCPP_INFO(this->get_logger(), "Found %u devices", count);
    bool master_found = false;

    for (auto it = devices.begin(); it != devices.end(); it++)
    {
        if (it->find("master") != std::string::npos && !master_found)
        {
            RCLCPP_INFO(this->get_logger(), "Found Master.");
            auto node_id = config_->get_entry<uint32_t>(*it, "node_id");
            auto driver_name = config_->get_entry<std::string>(*it, "driver");
            auto package_name = config_->get_entry<std::string>(*it, "package");

            if (!node_id.has_value() || !driver_name.has_value() || !package_name.has_value())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has uncomplete configuration for master");
                return false;
            }

            if (!this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it))
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Loading master failed.");
                return false;
            }
            add_node_to_executor(driver_name.value(), node_id.value(), *it);
            add_master(node_id.value());
            master_found = true;
        }
    }

    if (!master_found)
    {
        RCLCPP_ERROR(this->get_logger(), "Error: Master not in configuration");
        return false;
    }

    for (auto it = devices.begin(); it != devices.end(); it++)
    {
        if (it->find("master") == std::string::npos)
        {
            auto node_id = config_->get_entry<uint32_t>(*it, "node_id");
            auto driver_name = config_->get_entry<std::string>(*it, "driver");
            auto package_name = config_->get_entry<std::string>(*it, "package");

            if (!node_id.has_value() || !driver_name.has_value() || !package_name.has_value())
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has uncomplete configuration for %s", it->c_str());
                return false;
            }
            auto res = this->registered_drivers_.emplace(*it, std::make_pair(node_id.value(), driver_name.value()));
            if (!res.second)
            {
                RCLCPP_ERROR(this->get_logger(), "Error: Bus Configuration has duplicate configuration for %s", it->c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Found device %s with driver %s", it->c_str(), driver_name.value().c_str());

            this->load_component(package_name.value(), driver_name.value(), node_id.value(), *it);
            add_node_to_executor(driver_name.value(), node_id.value(), *it);
            this->add_driver_to_master(driver_name.value(), node_id.value());
        }
    }

    auto components = list_components();
    RCLCPP_INFO(this->get_logger(), "List of active components:");
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        RCLCPP_INFO(this->get_logger(), "%i : %s", it->first, it->second.c_str());
    }
    return true;
}

bool DeviceContainerNode::init()
{
    this->loadNode_srv_.reset();
    this->unloadNode_srv_.reset();

    this->get_parameter("can_interface_name", can_interface_name_);
    this->get_parameter("master_config", dcf_txt_);
    this->get_parameter("master_bin", dcf_bin_);
    this->get_parameter("bus_config", bus_config_);

    this->config_ = std::make_shared<ros2_canopen::ConfigurationManager>(bus_config_);
    this->config_->init_config();
    this->init_devices_from_config();
    return true;
}

void DeviceContainerNode::on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response)
{
    auto components = list_components();
    // RCLCPP_INFO(this->get_logger(), "List of active components:");
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        // RCLCPP_INFO(this->get_logger(), "%i : %s", it->first, it->second.c_str());
        response->unique_ids.push_back(it->first);
        response->full_node_names.push_back(it->second);
    }
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_manager = std::make_shared<DeviceContainerNode>(exec);
    std::thread spinThread([&device_manager]()
                           { 
                            if(device_manager->init())
                            {
                                RCLCPP_INFO(device_manager->get_logger(), "Initialisation successful.");
                            }
                            else
                            {
                                RCLCPP_INFO(device_manager->get_logger(), "Initialisation failed.");
                            } });
    exec->add_node(device_manager);
    exec->spin();
    spinThread.join();
    return 0;
}