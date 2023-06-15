Operation
=========

The ros2_canopen stack can be used in three different ways:

* standard nodes container
* managed nodes container
* ros2_control system interface with standard nodes


Simple nodes container
""""""""""""""""""""""""
The standard node container mode bundles the master and all slave driver nodes in one specialised
container called device container. All nodes are simple ROS 2 nodes and expose a publish and subscribe
as well as a service interfaces. Once the device container is started, all nodes are brought up
and ready to be used.

**Purpose**:
The simple nodes container is thought for applications where the user needs a simple and
easy to launch interface and does not need any realtime control capabilities as provided by
ros2_control.

Managed nodes container
""""""""""""""""""""""""""
The managed nodes container has the same properties as the standard nodes container.
The exception is, that all nodes are lifecycle nodes and there is a special node called
lifecycle manager. The user can use the lifecycle manager to control the lifecycle of
all nodes in the container.

**Purpose**:
The managed nodes container is thought for applications where the user wants to have
more runtime recovery options than killing and restarting the container.


ROS 2 control system interface
""""""""""""""""""""""""""""""
The ros2_control interface is currently build on top of the simple nodes container. In
addition to the standard nodes container the ros2_control system interface provides a
hardware interface that can be used to control the devices on the bus. Currently, three
different system interfaces are provided:

* canopen_ros2_control/CANopenSystem
* canopen_ros2_control/CIA402System
* canopen_ros2_control/RobotSystem

**Purpose**:
The ROS 2 control system interfaces are thought for control applications that require
low latencies.
