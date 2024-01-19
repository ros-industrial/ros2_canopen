Running Examples
================

In order to tryout the library a few examples are provided in the ``canopen_tests`` directory.
You can run them if you have :ref:`started the vcan0 interface <quick-start-setup-can-controller>`.

Service Interface
---------------------

.. code-block:: bash

    ros2 launch canopen_tests cia402_setup.launch.py


Managed Service Interface
-------------------------

.. code-block:: bash

    ros2 launch canopen_tests cia402_lifecycle_setup.launch.py

ROS2 Control
------------

Proxy Setup
,,,,,,,,,,,

.. code-block:: bash

   ros2 launch canopen_tests canopen_system.launch.py


CiA402 Setup
,,,,,,,,,,,,

.. code-block:: bash

   ros2 launch canopen_tests cia402_system.launch.py


Robot Setup
,,,,,,,,,,,,

.. code-block:: bash

    ros2 launch canopen_tests robot_control_setup.launch.py
