Operation of the ROS2 CANopen Stack
===================================

The ROS2 CANopen stack provides flexible operation modes tailored to various application requirements. It can be configured in one of the following three ways:

1. Simple Nodes Container
2. Managed Nodes Container
3. ROS 2 Control System Interface

Simple Nodes Container
----------------------
The Simple Nodes Container bundles the master with all slave driver nodes into a specialized container known as the device container. All nodes within
this container are simple ROS 2 nodes, offering publish, subscribe, and service interfaces. Once the device container is launched, all contained nodes
are immediately operational.

**Purpose**: This container is designed for scenarios requiring a straightforward, easy-to-launch interface without the need for real-time control,
such as those offered by *ros2_control*.

For more information on configuring and utilizing these interfaces, see :doc:`operation/service-interface`.

Managed Nodes Container
-----------------------
The Managed Nodes Container operates similarly to the Simple Nodes Container, with the key difference being the use of lifecycle nodes and a dedicated
lifecycle manager. This manager allows for sophisticated control over the lifecycle of all nodes in the container, enhancing fault recovery and system manageability.
For more information on lifecycle nodes, refer to the `ROS 2 Node Lifecycle documentation <https://design.ros2.org/articles/node_lifecycle.html>`_.

**Purpose**: Ideal for applications demanding enhanced runtime recovery options beyond simple restarts.

For more information on configuring and utilizing these interfaces, see :doc:`operation/managed-service-interface`.

ROS 2 Control System Interface
------------------------------
Built on the foundation of the Simple Nodes Container, the ROS 2 Control System Interface integrates a hardware interface that facilitates direct control
over devices connected on the bus. It supports several system interfaces designed for specific control needs:

- canopen_ros2_control/CANopenSystem
- canopen_ros2_control/CIA402System
- canopen_ros2_control/RobotSystem

**Purpose**: These interfaces are suitable for applications requiring precise, low-latency control mechanisms.

For more information on configuring and utilizing these interfaces, refer to :doc:`operation/ros2-control-interface`.

Requirements for Setting Up the ROS2 CANopen Stack
--------------------------------------------------
To start with ROS2 CANOpen Stack, you need to have the following prerequisites:

To effectively implement and utilize the ROS2 CANopen stack, certain prerequisites must be met. Below is a comprehensive list of these requirements:

1. **EDS or DCF Files**: You'll need the Electronic Data Sheet (EDS) or Device Configuration File (DCF) for each CANopen device. These files contain
    crucial device-specific parameters and configuration details necessary for communication and operational functionality.

2. **bus.yml File Configuration**: Prepare the ``bus.yml`` file, which outlines the bus topology and device-specific settings. This configuration file should specify details such as which devices are connected, the relevant EDS/DCF files, parameter overrides, and driver assignments for each device.

3. **Network Configuration**: Set up and configure your network to match the requirements of your CANopen devices and the ROS2 CANopen stack. See :doc:`../quickstart/setup-network`

These are the basic requirements, and continue with the following sections to learn more about the operation modes and how to set up the ROS2 CANopen stack.
