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

#ifndef DEVICE_HPP_
#define DEVICE_HPP_

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "canopen_core/configuration_manager.hpp"

using namespace lely;

namespace ros2_canopen
{
    // Base class for driver plugin
    // Pluginlib API does allows only default constructors
    class LifecycleDriverInterface : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        /**
         * @brief Construct a new LifecycleDriverInterface object
         *
         * @param [in] node_name
         * @param [in] node_options
         */
        LifecycleDriverInterface(const std::string &node_name,
                        const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions()) 
                        : rclcpp_lifecycle::LifecycleNode(node_name, node_options) {}

        /**
         * @brief Init driver with Master
         *
         * @param [in] exec         Executor that the driver will be added to
         * @param [in] master       Master that controls the driver
         * @param [in] node_id      Node ID of the driver
         */
        virtual void init_from_master(std::shared_ptr<ev::Executor> exec,
                          std::shared_ptr<canopen::AsyncMaster> master,
                          std::shared_ptr<ros2_canopen::ConfigurationManager> config) = 0;
        
        virtual void init() = 0;

        virtual bool add() = 0;
        virtual bool remove() = 0;
    protected:
        std::shared_ptr<ev::Executor> exec_;
        std::shared_ptr<canopen::AsyncMaster> master_;
        uint8_t node_id_;
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_;
    };

    class LifecycleMasterInterface : public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::string dcf_txt_;
        std::string dcf_bin_;
        std::string can_interface_name_;
        uint8_t node_id_;
        std::shared_ptr<ros2_canopen::ConfigurationManager> config_;

    public:
        /**
         * @brief Construct a new Master Interface object
         *
         * @param node_name
         * @param node_options
         * @param dcf_txt
         * @param dcf_bin
         * @param can_interface_name
         * @param nodeid
         */
        LifecycleMasterInterface(
            const std::string &node_name,
            const rclcpp::NodeOptions &node_options) : rclcpp_lifecycle::LifecycleNode(node_name, node_options)
        {
        }
        virtual void init()
        {
        }

        /**
         * @brief Initialises a driver
         *
         * @param node_id
         */
        virtual void init_driver(std::shared_ptr<ros2_canopen::LifecycleDriverInterface>, uint8_t node_id) = 0;

    };

} // end ros2_canopen namespace

#endif // DEVICE_HPP_