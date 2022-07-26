#include "canopen_core/device_manager_node.hpp"

namespace ros2_canopen
{

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    DeviceManagerNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        if (!this->has_parameter("container_name"))
        {
            this->declare_parameter<std::string>("container_name");
        }

        this->get_parameter<std::string>("container_name", this->container_name_);

        std::string add_client_name = this->container_name_ + "/add_driver_to_master";
        this->add_driver_client_ = this->create_client<canopen_interfaces::srv::CONode>(add_client_name);

        std::string remove_client_name = this->container_name_ + "/remove_driver_from_master";
        this->remove_driver_client_ = this->create_client<canopen_interfaces::srv::CONode>(remove_client_name);

        if (this->load_from_config())
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    DeviceManagerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        if (this->bring_up_all())
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    DeviceManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        if (this->bring_down_all())
        {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    DeviceManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    DeviceManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void
    DeviceManagerNode::init(std::shared_ptr<ros2_canopen::ConfigurationManager> config)
    {
        this->config_ = config;
    }

    bool
    DeviceManagerNode::load_from_config()
    {

        std::vector<std::string> devices;
        uint32_t count = this->config_->get_all_devices(devices);

        // Find master in configuration
        for (auto it = devices.begin(); it != devices.end(); it++)
        {
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr driver_get_state_clients;
            rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr driver_change_state_clients;
            uint8_t node_id = config_->get_entry<uint8_t>(*it, "node_id").value();
            std::string change_state_client_name = *it;
            std::string get_state_client_name = *it;
            get_state_client_name += "/get_state";
            change_state_client_name += "/change_state";

            device_names_to_ids.emplace(*it, node_id);
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(get_state_client_name);
            rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client = this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_client_name);
            this->drivers_get_state_clients.emplace(node_id, get_state_client);
            this->drivers_change_state_clients.emplace(node_id, change_state_client);

            if (it->find("master") != std::string::npos)
            {
                this->master_id_ = node_id;
            }
        }
        return true;
    }

    unsigned int
    DeviceManagerNode::get_state(uint8_t node_id, std::chrono::seconds time_out)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto client = this->drivers_get_state_clients[node_id];
        if (!client->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We send the service request for asking the current
        // state of the lc_talker node.
        auto future_result = client->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %hhu", node_id);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        // We have an succesful answer. So let's print the current state.
        if (future_result.get())
        {
            RCLCPP_INFO(
                get_logger(), "Node %hhu has current state %s.",
                node_id, future_result.get()->current_state.label.c_str());
            return future_result.get()->current_state.id;
        }
        else
        {
            RCLCPP_ERROR(
                get_logger(), "Failed to get current state for node %hhu", node_id);
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool
    DeviceManagerNode::change_state(uint8_t node_id, uint8_t transition, std::chrono::seconds time_out)
    {
        auto client = this->drivers_change_state_clients[node_id];
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;
        if (!client->wait_for_service(time_out))
        {
            RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                client->get_service_name());
            return false;
        }

        // We send the request with the transition we want to invoke.
        auto future_result = client->async_send_request(request);

        // Let's wait until we have the answer from the node.
        // If the request times out, we return an unknown state.
        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready)
        {
            RCLCPP_ERROR(
                get_logger(), "Server time out while getting current state for node %hhu", node_id);
            return false;
        }

        // We have an answer, let's print our success.
        if (future_result.get()->success)
        {
            RCLCPP_INFO(
                get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
            return true;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
            return false;
        }
        return false;
    }

    bool
    DeviceManagerNode::add_driver_to_master(uint8_t node_id, std::chrono::seconds time_out)
    {
        while (!add_driver_client_->wait_for_service(time_out))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
        request->nodeid = node_id;

        auto future_result = add_driver_client_->async_send_request(request);

        auto future_status = this->wait_for_result(future_result, time_out);

        if (future_status == std::future_status::ready)
        {
            try
            {
                return future_result.get()->success;
            }
            catch (...)
            {
                return false;
            }
        }
        return false;
    }

    bool
    DeviceManagerNode::remove_driver_from_master(uint8_t node_id, std::chrono::seconds time_out)
    {
        while (!remove_driver_client_->wait_for_service(time_out))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
        request->nodeid = node_id;


        auto future_result = remove_driver_client_->async_send_request(request);

        auto future_status = this->wait_for_result(future_result, time_out);

        if (future_status == std::future_status::ready)
        {
            try
            {
                return future_result.get()->success;
            }
            catch (...)
            {
                return false;
            }
        }
        return false;
    }

    bool
    DeviceManagerNode::bring_up_master()
    {
        auto state = this->get_state(master_id_);
        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Master not in unconfigured state.");
            return false;
        }

        if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Configure Transition failed.");
            return false;
        }

        if (!this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up master. Activate Transition failed.");
            return false;
        }
        return true;
    }

    bool
    DeviceManagerNode::bring_down_master()
    {
        this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        this->change_state(master_id_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        
        auto state = this->get_state(master_id_);

        if(state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            return false;
        }

        return true;
    }

    bool
    DeviceManagerNode::bring_up_driver(std::string device_name)
    {
        auto node_id = this->device_names_to_ids[device_name];
        auto master_state = this->get_state(master_id_);
        auto state = this->get_state(node_id);

        if (master_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Master not in active state.", device_name.c_str());
            return false;
        }

        if (state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Not in unconfigured state.", device_name.c_str());
            return false;
        }

        if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Configure Transition failed.", device_name.c_str());
            return false;
        }

        if (!this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s. Activate Transition failed.", device_name.c_str());
            return false;
        }

        if (!this->add_driver_to_master(node_id))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to add %s to masters event loop.", device_name.c_str());
            return false;
        }

        return true;
    }

    bool
    DeviceManagerNode::bring_down_driver(std::string device_name)
    {
        auto node_id = this->device_names_to_ids[device_name];
        auto master_state = this->get_state(master_id_);
        

        if (master_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            if (!this->remove_driver_from_master(node_id))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to remove %s to masters event loop.", device_name.c_str());
                return false;
            }
        }

        this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        this->change_state(node_id, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
        auto state = this->get_state(node_id);
        if(state != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
            return false;
        }
        return true;
    }

    bool
    DeviceManagerNode::bring_up_all()
    {
        if (!this->bring_up_master())
        {
            return false;
        }
        for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
        {
            if (it->first.find("master") != std::string::npos)
            {
                if (!this->bring_up_driver(it->first))
                {
                    return false;
                }
            }
        }
        return true;
    }

    bool
    DeviceManagerNode::bring_down_all()
    {
        for (auto it = this->device_names_to_ids.begin(); it != this->device_names_to_ids.end(); ++it)
        {
            if (it->first.find("master") != std::string::npos)
            {
                if (!this->bring_down_driver(it->first))
                {
                    return false;
                }
            }
        }
        if (!this->bring_down_master())
        {
            return false;
        }

        return true;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::DeviceManagerNode)