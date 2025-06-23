^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_base_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------
* Add timeouts
* Contributors: Vishnuprasad Prachandabhanu

0.3.1 (2025-06-23)
------------------
* Add boot timeout and retry
* Include driver exception when boot failed
* Boot Timeout: Add parameter to base driver to pass to wait as timeout
* Contributors: Gerry Salinas, Luis Camero, Vishnuprasad Prachandabhanu, ipa-vsp

0.3.0 (2024-12-12)
------------------

0.2.12 (2024-04-22)
-------------------
* Merge pull request `#280 <https://github.com/ros-industrial/ros2_canopen/issues/280>`_ from ipa-vsp/fix/yaml-build-error
  Fix undefined reference to yaml library
* pre-commit update
* Merge pull request `#261 <https://github.com/ros-industrial/ros2_canopen/issues/261>`_ from gsalinas/configurable-sdo-timeout
  Configurable SDO timeout
* 0.2.9
* forthcoming
* Merge pull request `#220 <https://github.com/ros-industrial/ros2_canopen/issues/220>`_ from ipa-vsp/feature/timeout-config
  Add timeouts
* Set SDO timeout from node config.
* Replace two more hardcoded timeouts.
* Include timeout in documentation comment for LelyDriverBridge.
* Make 20ms a default argument of the master & driver bridges.
* change error to warn for testing
* remove build warings
* non transmit time from bus.yml
* Contributors: Gerry Salinas, Vishnuprasad Prachandabhanu, ipa-vsp

0.2.8 (2024-01-19)
------------------

0.2.7 (2023-06-30)
------------------
* Add missing license headers and activate ament_copyright
* Fix maintainer naming
* Update printed output in lely_driver_bridge.cpp
* Contributors: Christoph Hellmann Santos, yos627

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
* Fix base driver lifecyle
* Fix node polling mode in base driver
* Contributors: Błażej Sowa, Christoph Hellmann Santos

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro, Vishnuprasad Prachandabhanu
