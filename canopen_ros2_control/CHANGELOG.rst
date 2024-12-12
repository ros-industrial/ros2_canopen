^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------

0.3.0 (2024-12-12)
------------------
* pre-commit fix
* impl operation mode
* Add cyclic torque mode to cia402 driver and robot system controller (`#293 <https://github.com/ros-industrial/ros2_canopen/issues/293>`_)
  * Add base functions for switching to cyclic torque mode
  * Add cyclic torque mode as effort interface to robot_system controller
  * Add documentation about cyclic torque mode.
  ---------
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
