ros2_control SystemInterface test
=================================

Test details
------------

.. csv-table:: Tests
    :header: "Detail", "Information"
    :delim: ;

    Package; canopen_tests
    Test file; launch/canopen_system.launch.py
    Description; Create an exemplary ros2_control SystemInterface with CAN master and communicates to a slave node.
    Prerequisites; vcan0 must be available

To bring up vcan0:

   .. code-block:: bash

      sudo modprobe vcan
      sudo ip link add dev vcan0 type vcan
      sudo ip link set vcan0 txqueuelen 1000
      sudo ip link set up vcan0


Explanation of the test
------------------------

The test starts generic system interface and generic proxy controller for CanOpen devices.
Generic system interface enables integration of values from the CAN Bus into ros2_control framework and the controller enables you to send and receive data from the CAN bus through ros2_control to ROS2.

The next few lines show you some command to have exemplary usage of the ros2_control integration:

1. After the test is started check ``/dynamic_joint_states`` to show internal data from CAN nodes in ros2_control system.

   .. code-block:: bash

      ros2 topic echo /dynamic_joint_states


2. Open a new terminal and echo data from the topic ``/node_1_controller/rpdo``.

   .. code-block:: bash

      ros2 topic echo /node_1_controller/rpdo


3. Open a new terminal reset the state of nmt.

   .. code-block:: bash

      ros2 service call /node_1_controller/nmt_reset_node std_srvs/srv/Trigger {}

   You will expect changes in the ``/dynamic_joint_states`` topic.


4. In a new terminal publish some data to the controller to write them to the CAN bus:

   .. code-block:: bash

      ros2 topic pub --once /node_1_controller/tpdo canopen_interfaces/msg/COData "
      index: 0x4000
      subindex: 0
      data: 0x1122"

   Now watch how data in the topic ``/node_1_controller/rpdo`` are changing. There is a mirror of the data on 0x4001.
   That is, the slave node will mirror the data on 0x4001 via its tpdo and the proxy device will get the data via its rpdo.
   You should see this

   .. code-block:: bash

      index: 16385  # This is 0x4001
      subindex: 0
      data: 4386
      ---
