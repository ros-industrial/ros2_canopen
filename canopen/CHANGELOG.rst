^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------
* Update maintainer list
* Update package.xml
* Add timeouts
* Contributors: Vishnuprasad Prachandabhanu

0.3.1 (2025-06-23)
------------------
* Sync upstream 'master' into homing_timeout_pr.
  Fix homing info message to include both offset
  and homing timeout information.
* Add namespacing support
* Contributors: Vishnuprasad Prachandabhanu

0.3.0 (2024-12-12)
------------------
* Update CiA402 bus config docs
* Remove set heartbeat service from master documentation (`#294 <https://github.com/ros-industrial/ros2_canopen/issues/294>`_)
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* Add cyclic torque mode to cia402 driver and robot system controller (`#293 <https://github.com/ros-industrial/ros2_canopen/issues/293>`_)
  * Add base functions for switching to cyclic torque mode
  * Add cyclic torque mode as effort interface to robot_system controller
  * Add documentation about cyclic torque mode.
  ---------
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>

0.2.12 (2024-04-22)
-------------------
* Merge pull request `#280 <https://github.com/ros-industrial/ros2_canopen/issues/280>`_ from ipa-vsp/fix/yaml-build-error
  Fix undefined reference to yaml library
* pre-commit update
* Merge pull request `#261 <https://github.com/ros-industrial/ros2_canopen/issues/261>`_ from gsalinas/configurable-sdo-timeout
  Configurable SDO timeout
* 0.2.9
* forthcoming
* Merge pull request `#272 <https://github.com/ros-industrial/ros2_canopen/issues/272>`_ from ros-industrial/ipa-vsp-patch-1
  Update maintainer list
* Update package.xml
* Merge pull request `#220 <https://github.com/ros-industrial/ros2_canopen/issues/220>`_ from ipa-vsp/feature/timeout-config
  Add timeouts
* Add SDO timeout to device config documentation.
* change from 100ms to 2000ms
* timeout for booting slave
* Contributors: Gerry Salinas, Vishnuprasad Prachandabhanu, ipa-vsp

0.2.8 (2024-01-19)
------------------
* Documentation: Fix launch file spelling (`#208 <https://github.com/ros-industrial/ros2_canopen/issues/208>`_)
* Documentation: Fix package creation command.  (`#176 <https://github.com/ros-industrial/ros2_canopen/issues/176>`_)
  * Fix package creation
  * Fixed bus.yml nodes list
  * docs: fixed launch file path in instructions
  ---------
  Co-authored-by: waterlinux <water@gmail.com>
* Contributors: Lewe Christiansen, Vishnuprasad Prachandabhanu

0.2.7 (2023-06-30)
------------------
* Update ros2_control docs.
* Contributors: Christoph Hellmann Santos, Dr. Denis, Xi Huang

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

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Borong Yuan, Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Vishnuprasad Prachandabhanu
