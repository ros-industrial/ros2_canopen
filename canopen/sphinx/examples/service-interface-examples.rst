Service Interface Examples
==========================

Before running these examples, ensure that to :ref:`start the vcan0 interface <quick-start-setup-can-controller>`. 
You can find these examples in the ``canopen_test`` package.

Proxy Driver Examples
---------------------
A proxy driver provides a simple way to forward CANopen functionalities for a specific device using ROS2 services and messages. For more details on the proxy driver, see :doc:`../user-guide/proxy-driver`.

Configuring ``bus.yaml``
~~~~~~~~~~~~~~~~~~~~~~~~
The configuration for the proxy driver is specified in the ``bus.yaml`` file. Here's an example configuration:

.. code-block:: yaml
    
    options:
        dcf_path: "@BUS_CONFIG_PATH@"

    master:
        node_id: 1
        driver: "ros2_canopen::MasterDriver"
        package: "canopen_master_driver"

    defaults:
        dcf: "simple.eds"
        driver: "ros2_canopen::ProxyDriver"
        package: "canopen_proxy_driver"
        polling: true
        period: 10

    nodes:
        proxy_device_1:
            node_id: 2
        proxy_device_2:
            node_id: 3

For further details about the configuration, refer to :doc:`../user-guide/configuration`. Additionally, add ``cogen_dcf(simple)`` in the *CMakeLists.txt* file to generate the necessary artifacts for the proxy driver.

Running the Proxy Driver Examples
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
To launch the proxy driver setup, use the following command:

.. code-block:: bash

    $ ros2 launch canopen_tests proxy_setup.launch.py

Verify that all services are available by running the ROS2 topic and service list commands. To conduct a test:

.. code-block:: bash

    $ ros2 topic pub /proxy_device_1/tpdo canopen_interfaces/msg/COData "{ index: 0x4000, subindex: 0, data: 200 }"

Check the output by running:

.. code-block:: bash

    $ ros2 topic echo /proxy_device_1/rpdo

.. note::

    These examples use a fake slave device. Running on actual hardware may yield different results. 
    Always refer to the device's documentation for specific details.

CiA402 Driver Examples
-----------------------