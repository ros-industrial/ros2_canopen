^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_402_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------
* Update the lely_core_libraries hash to the latest.
* fix ci build error
* Contributors: Vishnuprasad Prachandabhanu

0.3.0 (2024-12-12)
------------------
* Reformat using pre-commit
* Implement position offsets
* pre-commit fix
* impl operation mode
* Add cyclic torque mode to cia402 driver and robot system controller (`#293 <https://github.com/ros-industrial/ros2_canopen/issues/293>`_)
  * Add base functions for switching to cyclic torque mode
  * Add cyclic torque mode as effort interface to robot_system controller
  * Add documentation about cyclic torque mode.
  ---------
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>

0.2.12 (2024-04-22)
-------------------
* 0.2.9
* forthcoming
* Merge pull request `#267 <https://github.com/ros-industrial/ros2_canopen/issues/267>`_ from clalancette/clalancette/update-lely-core-hash
  Update the lely_core_libraries hash to the latest.
* fix ci build error
* Contributors: Vishnuprasad Prachandabhanu, ipa-vsp

0.2.8 (2024-01-19)
------------------

0.2.7 (2023-06-30)
------------------
* Add missing license headers and activate ament_copyright
* Fix maintainer naming
* Contributors: Christoph Hellmann Santos

0.2.6 (2023-06-24)
------------------

0.2.5 (2023-06-23)
------------------

0.2.4 (2023-06-22)
------------------

0.2.3 (2023-06-22)
------------------

0.2.2 (2023-06-21)
------------------

0.2.1 (2023-06-21)
------------------
* Fix boost/std placeholders ambiguity in older boost versions
* Contributors: Christoph Hellmann Santos

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Borong Yuan, Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, G.A. vd. Hoorn, Lovro, Vishnuprasad Prachandabhanu, livanov93
