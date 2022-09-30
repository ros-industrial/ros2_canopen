#ifndef DRIVER_NODE_HPP_
#define DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"

namespace ros2_canopen
{

    class CanopenDriverInterface
    {
    public:
        virtual void init() = 0;
        virtual void set_master(std::shared_ptr<lely::ev::Executor> exec,
                                std::shared_ptr<lely::canopen::AsyncMaster> master) = 0;
        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;
        virtual void shutdown() = 0;
    };

    class CanopenDriver : public CanopenDriverInterface, public rclcpp::Node
    {
    public:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;
        explicit CanopenDriver(
            const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : rclcpp::Node("canopen_driver", node_options)
        {
            node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp::Node>>(this);
        }

        virtual void init() override;

        virtual void set_master(
            std::shared_ptr<lely::ev::Executor> exec,
            std::shared_ptr<lely::canopen::AsyncMaster> master) override;

        virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp::Node::get_node_base_interface();
        }

        virtual void shutdown() override;
    };

    class LifecycleCanopenDriver : public CanopenDriverInterface, public rclcpp_lifecycle::LifecycleNode
    {
    protected:
        std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;

    public:
        explicit LifecycleCanopenDriver(
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
            : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_driver", node_options)
        {
            node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp_lifecycle::LifecycleNode>>(this);
        }

        virtual void init() override;

        virtual void set_master(
            std::shared_ptr<lely::ev::Executor> exec,
            std::shared_ptr<lely::canopen::AsyncMaster> master) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
        {
            return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
        }
        virtual void shutdown() override;
    };

}

#endif