^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------
* Add timeouts
* Contributors: Vishnuprasad Prachandabhanu, ipa-vsp

0.3.1 (2025-06-23)
------------------
* Add namespacing support
* Contributors: Christoph Hellmann Santos, Gerry Salinas, Vishnuprasad Prachandabhanu, ipa-vsp

0.3.0 (2024-12-12)
------------------

0.2.12 (2024-04-22)
-------------------
* Merge pull request `#270 <https://github.com/ros-industrial/ros2_canopen/issues/270>`_ from gsalinas/can-namespace-pr
  Put components loaded by the device container into its namespace, if any.
* pre-commit update
* Put components loaded by the device container into its namespace, if any.
* Merge pull request `#280 <https://github.com/ros-industrial/ros2_canopen/issues/280>`_ from ipa-vsp/fix/yaml-build-error
  Fix undefined reference to yaml library
* fix undefined reference to yaml
* Fix logging in device_container.cpp (`#277 <https://github.com/ros-industrial/ros2_canopen/issues/277>`_)
* 0.2.9
* forthcoming
* Merge pull request `#220 <https://github.com/ros-industrial/ros2_canopen/issues/220>`_ from ipa-vsp/feature/timeout-config
  Add timeouts
* change from 100ms to 2000ms
* non transmit time from bus.yml
* timeout for booting slave
* Contributors: Christoph Hellmann Santos, Gerry Salinas, Vishnuprasad Prachandabhanu, ipa-vsp

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
* Fix master and driver lifecycle
* Fix QoS build warning in canopen_core
* Use consistenlty HEX output for NodeID and Index.
* Contributors: Christoph Hellmann Santos, Denis Štogl, Vishnuprasad Prachandabhanu

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Aulon Bajrami, Borong Yuan, Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, Lovro, Vishnuprasad Prachandabhanu
