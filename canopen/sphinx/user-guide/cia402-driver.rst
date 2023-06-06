Cia402 Driver
========================

The Cia402 Driver implements the CIA402 profile for motion controllers and enables setting
the drive status, operation mode and sending target values to the motion controller.

.. csv-table:: CIA402 Drivers
   :header: Type, Package, Name
   :widths: 30, 20, 50

   lifecycle, canopen_402_driver, ros2_canopen::LifecycleCia402Driver
   simple, canopen_402_driver, ros2_canopen::Cia402Driver

Services
--------

.. list-table::
  :widths: 30 20 50
  :header-rows: 1
  :align: left

  * - Services
    - Type
    - Description
  * - ~/nmt_reset_node
    - Trigger
    - Resets CANopen Device the Proxy Device Node manages.
  * - ~/sdo_read
    - CORead
    - Reads an SDO object from the specified index, subindex and datatype of the remote device.
  * - ~/sdo_write
    - COWrite
    - Writes data to an SDO object on the specified index, subindex and datatype of the remote device.
  * - ~/init
    - Trigger
    - Initialises motion controller including referencing
  * - ~/recover
    - Trigger
    - Recovers motion controller
  * - ~/halt
    - Trigger
    - Stops motion controller
  * - ~/position_mode
    - Trigger
    - Switches to profiled position mode
  * - ~/velocity_mode
    - Trigger
    - Switches to profiled velocity mode
  * - ~/torque_mode
    - Trigger
    - Switches to profiled torque mode
  * - ~/cyclic_position_mode
    - Trigger
    - Switches to cyclic position mode
  * - ~/cyclic_velocity_mode
    - Trigger
    - Switches to cyclic velocity mode
  * - ~/interpolated_position_mode
    - Trigger
    - Switches to interpolated position mode, only linear mode with fixed time is supported
  * - ~/target
    - CODouble
    - Sets the target value. Only accepted when an operation mode is set.

Publishers
----------
.. list-table::
  :widths: 30 20 50
  :header-rows: 1
  :align: left

  * - Publishers
    - Type
    - Description
  * - ~/joint_states
    - sensor_msgs/msg/JointState
    - Joint states of the drive
  * - ~/nmt_state
    - String
    - Publishes NMT state on change
  * - ~/rpdo
    - COData
    - Publishes received PDO objects on reception


Subscribers
-----------

.. list-table::
  :widths: 30 20 50
  :header-rows: 1

  * - Topic
    - Type
    - Description
  * - ~/target
    - COTargetDouble
    - Sets target value.
  * - ~/tpdo
    - COData
    - Writes received data to remote device if the specified object is RPDO mapped on remote device.

Bus Configuration Parameters
----------------------------
Additional parameters that can be used in bus.yml for this driver.


.. list-table::
  :widths: 30 20 50
  :header-rows: 1

  * - Parameter
    - Type
    - Description
  * - polling
    - bool
    - Enables polling of the drive status. Default: true. If false, period will be used to run a ros2 timer as update loop. If true, the update loop will be triggered by the sync signal and directly executed in the canopen realtime loop. This requires all data processed in the update loop to be PDO, otherwise the loop will get stuck. This can speed reduce processor load significantly though.
  * - period
    - Milliseconds
    - Refresh period for 402 state machine. Should be similar to sync period of master.
  * - switching_state
    - see below
    - The state to switch the operation mode in.
  * - scale_pos_to_dev
    - double
    - Scaling factor to convert from SI units to device units for position.
  * - scale_vel_to_dev
    - double
    - Scaling factor to convert from SI units to device units for velocity.
  * - scale_pos_from_dev
    - double
    - Scaling factor to convert from device units to SI units for position.
  * - scale_vel_from_dev
    - double
    - Scaling factor to convert from device units to SI units for velocity.
  * - position_mode
    - int
    - The drives operation mode to use for the position interface
  * - velocity_mode
    - int
    - The drives operation mode to use for the velocity interface
  * - torque_mode
    - int
    - The drives operation mode to use for the torque interface
