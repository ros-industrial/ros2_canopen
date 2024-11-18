Hardware Interface
------------------
This package provides multiple hardware interfaces for testing. Mainly the following:

- canopen_ros2_control/CanopenSystem: A system interface for ProxyDrivers
- canopen_ros2_control/Cia402System: A system interface for Cia402Drivers
- canopen_ros2_control/RobotSystem: A system interface for Cia402Drivers in a robot configuration.


Robot System Interface
----------------------
The Robot System Interface utilizes information from the robot's URDF (Unified Robot Description Format) to configure
and manage Cia402Drivers via the ros2_control hardware interface. The system configuration is determined by the ``bus.yml``
file, and each joint's associated CANopen device is specified using the ``node_id``.

**Configuration Example**:

.. code-block:: xml

    <ros2_control name="${name}" type="system">
        <hardware>
            <plugin>canopen_ros2_control/RobotSystem</plugin>
            <param name="bus_config">[path to bus.yml]</param>
            <param name="master_config">[path to master.dcf]</param>
            <param name="can_interface_name">[can interface to be used]</param>
            <param name="master_bin">[master.bin if it exists]</param>
        </hardware>
        <joint name="joint1">
            <param name="node_id">3</param>
            ...
        </joint>
        <joint name="joint2">
            <param name="node_id">4</param>
            ...
        </joint>
    </ros2_control>

.. note::
    For practical implementation examples, refer to the `canopen_tests` package.

ROS2 Controllers
----------------
The package provides several controllers optimized for different setups within the ROS2 framework:

- **canopen_ros2_controllers/Cia402RobotController**: Integrates seamlessly with the Robot System Interface.
- **canopen_ros2_controllers/Cia402DeviceController**: Compatible with the Cia402System, facilitating device-specific controls.
- **canopen_ros2_controllers/CanopenProxyController**: Works with both CanopenSystem and Cia402System interfaces, providing versatile control options.

Robot Controller Configuration
------------------------------
The Robot Controller simplifies the operation of robotic joints, automatically managing their states through the ros2_controller lifecycle.
Once activated, the controller ensures that all drives are operational without requiring further user intervention.

**Configuration Parameters**:

.. code-block:: yaml

    robot_controller:
        ros__parameters:
            joints:  # List of joints controlled by the controller
                - joint1
                - joint2
            operation_mode: 1  # Operational mode of the controller
            command_poll_freq: 5  # Frequency (Hz) at which the controller polls for command feedback
