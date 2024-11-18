Setup CAN Controller
====================
.. _quick-start-setup-can-controller:

To interact with the CAN controller you can use ``can-utils`` in order to know the output of the controller.

.. code-block:: console

  $ sudo apt install can-utils
  $ candump

.. note::
  This step is only required in order to know the output of the controller. One can skip this step to proceed with the setup of the controller.

**Option 1**: Virtual CANController

.. code-block:: console

  $ sudo modprobe vcan
  $ sudo ip link add dev vcan0 type vcan
  $ sudo ip link set vcan0 txqueuelen 1000
  $ sudo ip link set up vcan0


**Option 2**: Peak CANController

.. code-block:: console

  $ sudo modprobe peak_usb
  $ sudo ip link set can0 up type can bitrate 1000000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 3**: candleLight USB-CAN Adapter

.. code-block:: console

  $ sudo modprobe gs_usb
  $ sudo ip link set can0 up type can bitrate 500000
  $ sudo ip link set can0 txqueuelen 1000
  $ sudo ip link set up can0

Bitrate depends on your bus and devices capabilities.

**Option 4**: Adapt these steps to other socketcan devices
