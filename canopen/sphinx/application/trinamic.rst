Trinamic Stepper Motor control
==============================

Introduction
------------

This tutorial is a simple example of how to use the ros2_canopen package to control a `Trinamic smart stepper motor <https://www.trinamic.com/products/drives/details/pd42-x-1270/>`_.

Getting started
---------------

If you haven't already done so, follow the steps in the :doc:`../user-guide/configuration`.

Configuration
-------------

- Create a new folder in the ``config`` folder of your configuration package. Name it ``single-pd42``.
- Download ``.eds`` file from `Trinamic <https://www.trinamic.com/fileadmin/assets/Products/Drives_Software/TMCM-1270_CANopen_V326.zip>`_ and place ``TMCM-1270.eds`` in the ``single-pd42`` folder.
- Create a ``bus.yml`` file in the ``single-pd42`` folder with the following content:

    .. code-block:: yaml

        options:
            dcf_path: "@BUS_CONFIG_PATH@"

        master:
            node_id: 2
            driver: "ros2_canopen::MasterDriver"
            package: "canopen_master_driver"
            sync_period: 20000


        defaults:
            dcf: "TMCM-1270.eds"
            driver: "ros2_canopen::Cia402Driver"
            package: "canopen_402_driver"
            polling: false
            heartbeat_producer: 1000 # Heartbeat every 1000 ms
            sdo: # SDO executed during config
                - {index: 0x6081, sub_index: 0, value: 500000} # Set velocity
                - {index: 0x6083, sub_index: 0, value: 1000000} # Set acceleration
                - {index: 0x6083, sub_index: 0, value: 1000000} # Set deceleration
                - {index: 0x6085, sub_index: 0, value: 1000000} # Set quickstop deceleration
                - {index: 0x6098, sub_index: 0, value: 35} # Set default homing mode to 35
                - {index: 0x60C2, sub_index: 1, value: 50} # Set interpolation time for cyclic modes to 50 ms
                - {index: 0x60C2, sub_index: 2, value: -3} # Set base 10-3s

        nodes:
            trinamic_pd42:
                node_id: 1


- Edit the ``CMakeLists.txt`` file in the ``config`` folder of your configuration package and add the following lines:

    .. code-block:: cmake

        cmake_minimum_required(VERSION 3.8)
        project(trinamic_pd42_can)

        if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
        endif()

        # find dependencies
        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(std_srvs REQUIRED)
        find_package(canopen REQUIRED)
        find_package(lely_core_libraries REQUIRED)
        find_package(canopen_interfaces REQUIRED)

        # generate_dcf(single-pd42)
        cogen_dcf(single-pd42)

        add_executable(position_tick_client src/position_tick_motor.cpp)
        ament_target_dependencies(position_tick_client
        rclcpp std_srvs canopen_interfaces)


        install(TARGETS
        position_tick_client
        DESTINATION lib/${PROJECT_NAME})

        # install launch file
        install(DIRECTORY
        launch/
        DESTINATION share/${PROJECT_NAME}
        )

        if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        endif()

        ament_package()

- Create launch file in folder ``launch`` and add the following content:

    .. code-block:: python

        import os
        import sys

        import launch
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from ament_index_python import get_package_share_directory
        from launch import LaunchDescription


        def generate_launch_description():
            ld = LaunchDescription()
            slave_eds_path = os.path.join(
                get_package_share_directory("trinamic_pd42_can"), "config", "single-pd42", "TMCM-1270.eds"
            )

            slave_node_1 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                        "/cia402_slave.launch.py",
                    ]
                ),
                launch_arguments={
                    "node_id": "1",
                    "node_name": "pd42_slave",
                    "slave_config": slave_eds_path,
                }.items(),
            )
            master_bin_path = os.path.join(
                get_package_share_directory("trinamic_pd42_can"),
                "config",
                "single-pd42",
                "master.bin",
            )
            if not os.path.exists(master_bin_path):
                master_bin_path = ""

            device_container = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(get_package_share_directory("canopen_core"), "launch"),
                        "/canopen.launch.py",
                    ]
                ),
                launch_arguments={
                    "master_config": os.path.join(
                        get_package_share_directory("trinamic_pd42_can"),
                        "config",
                        "single-pd42",
                        "master.dcf",
                    ),
                    "master_bin": master_bin_path,
                    "bus_config": os.path.join(
                        get_package_share_directory("trinamic_pd42_can"),
                        "config",
                        "single-pd42",
                        "bus.yml",
                    ),
                    "can_interface_name": "vcan0",
                }.items(),
            )

            ld.add_action(device_container)
            ld.add_action(slave_node_1)

            return ld

Running the example
-------------------

To begin, follow the instructions for :doc:`../quickstart/operation`, which can be done using either a virtual or peak CAN interface.

If you prefer to use a real CAN interface, you will need to modify the launch file by changing the ``can_interface_name`` argument to ``can0``.
Additionally, if you are using real hardware, you should comment out the fake slave launch by adding a *#* in front of the line *ld.add_action(slave_node_1)*.
Once these changes have been made, you can launch the example.

.. code-block:: console

    ros2 launch trinamic_pd42_can <your launch file>.launch.py

Initilaize the motor by calling the service ``/trinamic_pd42/init``:

.. code-block:: console

    ros2 service call /trinamic_pd42/init std_srvs/srv/Trigger

Set the operation mode to ``Profile Position Mode`` by calling the service ``/trinamic_pd42/position_mode``:

.. code-block:: console

    ros2 service call /trinamic_pd42/position_mode std_srvs/srv/Trigger

Set the target to the motor by calling the service ``/trinamic_pd42/target``:

.. code-block:: console

    ros2 service call /trinamic_pd42/target canopen_interfaces/srv/COTargetDouble "{ target: 10.0 }"

Reference
---------
You can find the source code for this example in the `trinamic_pd42_can <https://github.com/ipa-cmh/trinamic_pd42_can.git>`_ package.
