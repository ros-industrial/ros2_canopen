Master Driver
=============

The master driver handles the creation of the necessary can interface and sets up a canopen event loop which drivers can hook onto.
In addition, the node offers services for communicating with nodes via nmt and sdo.

.. csv-table:: Master Drivers
   :header: Type, Package, Name
   :widths: 30, 20, 50

   lifecycle, canopen_master_driver, ros2_canopen::LifecycleMasterDriver
   simple, canopen_master_driver, ros2_canopen::MasterDriver


Services
--------

.. list-table::
  :widths: 30 20 50
  :header-rows: 1

  * - Services
    - Type
    - Description
  * - ~/read_sdo
    - COReadID
    - Reads an SDO object specified by Index, Subindex and Datatype of the device with the specified nodeid.
  * - ~/write_sdo
    - COWriteID
    - Writes Data to an SDO object specified by Index, Subindex and Datatype on the device with the specified nodeid.
  * - ~/set_heartbeat
    - COHeartbeatID
    - Sets the heartbeat of the device with the specified nodeid to the heartbeat value (ms)
  * - ~/set_nmt
    - CONmtID
    - Sends the NMT command to the device with the specified nodeid
