^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.2 (2025-12-05)
------------------
* add pdo 6077 torque actual value to the joint state interface as effort (`#316 <https://github.com/ros-industrial/ros2_canopen/issues/316>`_)
  Co-authored-by: Vishnuprasad Prachandabhanu <32260301+ipa-vsp@users.noreply.github.com>
* Refactor on_init method for improved readability and consistency
* Fix deprecated hardware_interface API (`#386 <https://github.com/ros-industrial/ros2_canopen/issues/386>`_)
* Fix typos in warning messages and comments for clarity
* `#379 <https://github.com/ros-industrial/ros2_canopen/issues/379>`_: Fix data conversion in the Lely Bridge to enable more data types and proper handling of Emcy in ros2_control
* Return error on Emcy.
* Add correct data conversion for Emcy.
* Fixed types handling in canopen_ros2_control.
* Optimize debug output.
* Fixed sending values.
* Contributors: Christoph Fröhlich, Dr. Denis Stogl, Vishnuprasad Prachandabhanu, synsi23b

0.3.1 (2025-06-23)
------------------
* Fixing ID type in storage of ros2_control system.
* Contributors: Dr. Denis, Gerry Salinas, Vishnuprasad Prachandabhanu

0.3.0 (2024-12-12)
------------------
* pre-commit fix
* impl operation mode
* Add cyclic torque mode to cia402 driver and robot system controller (`#293 <https://github.com/ros-industrial/ros2_canopen/issues/293>`_)
  * Add base functions for switching to cyclic torque mode
  * Add cyclic torque mode as effort interface to robot_system controller
  * Add documentation about cyclic torque mode.
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* Fix clang format
* Update canopen_system.hpp
* Add pdo mapping support
* Fix clang format
* Fix pre-commit
* Periodic messages sent, but not received properly.
* Fix the bug that the rpdo queue keeps poping although it is empty.
* Use proper function to get rpdo data
* Fix bug in state interface indexing..
* WIP: Extend the rpdo to have a queue (FIFO) to save the values.
  The read function take the latest value out of the queue and assign to the system interface.
  Need tests.

0.2.12 (2024-04-22)
-------------------
* 0.2.9
* forthcoming
* Contributors: ipa-vsp

0.2.9 (2024-04-16)
------------------

0.2.8 (2024-01-19)
------------------
* Update robot_system.cpp (`#168 <https://github.com/ros-industrial/ros2_canopen/issues/168>`_)
* Contributors: Christoph Hellmann Santos

0.2.7 (2023-06-30)
------------------
* Correct Proxy controller after changes and update tests.
* Contributors: Dr. Denis, Christoph Hellmann Santos

0.2.6 (2023-06-24)
------------------

0.2.5 (2023-06-23)
------------------

0.2.4 (2023-06-22)
------------------

0.2.3 (2023-06-22)
------------------
* Solve buildfarm issues
* Contributors: Christoph Hellmann Santos

0.2.2 (2023-06-21)
------------------

0.2.1 (2023-06-21)
------------------
* Use consistenlty (uppercase) HEX output for NodeID and Index.
* Contributors: Christoph Hellmann Santos, Denis Štogl

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro, Vishnuprasad Prachandabhanu, livanov93
