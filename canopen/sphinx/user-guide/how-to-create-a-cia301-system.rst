How to create a cia301 system with ros2_control
=================================================

The CiA 301 profile, also known as the CANopen application layer,
is the foundation of the CANopen protocol. It defines the basic principles,
communication services, and object dictionary used for device communication
and network management in a CANopen system.

The CiA 301 profile defines several communication services that allow devices
to exchange data and perform actions. These services include
the SDO (Service Data Object) service for transferring object data between devices,
the PDO (Process Data Object) service for real-time data exchange,
and the NMT (Network Management) service for network initialization,
device state control, and error handling.

To bring up the CiA301 interface using ROS2, it includes three steps:

- preparing the configuration for bus and ``ros2_control``
- preparing the state and command interfaces,
- and finally preparing the launch file.


Preparing the configuration
--------------------------------------------------------
To use the control system interface for CiA301 profile, we should prepare the following

- bus_conf
- master_dcf
- master_bin
- can_interface, (default: vcan0)

Define the bus configuration parameters

.. code-block:: yaml

    master:
        node_id: 1
        driver: "ros2_canopen::MasterDriver"
        package: "canopen_master_driver"
        baudrate: 250
    options:
        dcf_path: "@BUS_CONFIG_PATH@"
    joint_1:
        node_id: 0x00
        dcf: "joint.eds"
        driver: "ros2_canopen::ProxyDriver"
        package: "canopen_proxy_driver"
        reset_communication: false
    joint_2:
        node_id: 0x01
        dcf: "joint.eds"
        driver: "ros2_canopen::ProxyDriver"
        package: "canopen_proxy_driver"
        reset_communication: false

Define the ``ros2_control`` parameters

.. code-block:: yaml

    controller_manager:
        ros__parameters:
            update_rate: 100  # Hz

            joint_state_broadcaster:
                type: joint_state_broadcaster/JointStateBroadcaster

            joint_1_controller:
                type: canopen_ros2_controllers/CanopenProxyController

            joint_2_controller:
                type: canopen_ros2_controllers/CanopenProxyController

        joint_1_controller:
            ros__parameters:
                joint: joint_1


       joint_2_controller:
            ros__parameters:
                joint: joint_2

The example of master_dcf see https://github.com/ros-industrial/ros2_canopen/blob/master/canopen_tests/config/simple/simple.eds


Use RPDO to access the current state
--------------------------------------------------------
Defining the Joint and CANopen Data Structure
 The first step is to define a structure that will hold information about your joints and the associated CANopen data. This structure serves as the foundation for  your CANopen network, ensuring that all the relevant data is stored and accessible when needed.

PDO Index and Subindex
 Each Process Data Object (PDO) has an index and a subindex. The index acts as a unique identifier for each PDO, differentiating it from other PDOs in the system. The subindex is used to access individual data fields within each PDO as a PDO can contain multiple data fields.

Network Management (NMT)
 Network Management (NMT) is a fundamental service in the CANopen protocol suite. It offers basic device control commands such as start, stop, and reset, and manages the state of devices within the network.

 For RPDOs, the data are defined using:

 - "rpdo/index"
 - "rpdo/subindex"
 - "rpdo/type"
 - "rpdo/data"

 For NMT, we can read the states via:

 - "nmt/state"


Use TPDO to send commands
----------------------------
In order to send commands to hardware devices in a CANopen network, we first need to export the appropriate hardware interfaces. This is a critical step that enables us to effectively control each joint within our network.

Registering Transmit Process Data Objects (TPDOs)
 Similar to how we handle state interfaces, we must register Transmit Process Data Objects (TPDOs) for each joint. These TPDOs are related to the following commands:

- "tpdo/index"
- "tpdo/subindex"
- "tpdo/type"
- "tpdo/data"
- "tpdo/owns"

Network Management (NMT) Commands
 Beyond this, we have the ability to register commands associated with Network Management (NMT) to control the state of devices within our network. This is important for the smooth operation and control of our devices. The NMT related commands include:

- "nmt/reset"
- "nmt/reset_fbk"
- "nmt/start"
- "nmt/start_fbk"

These NMT commands not only help in managing the state of devices but also in providing feedback (indicated by "fbk") from the device to the control system after the execution of a command. This feedback mechanism is crucial for ensuring the successful execution of commands and managing the overall health of the network.


How to launch the nodes
----------------------------
Finally, we prepare the launch file for the interface. An example see: https://github.com/ros-industrial/ros2_canopen/blob/master/canopen_ros2_control/launch/canopen_system.launch.py

For testing, please refer to following section.
