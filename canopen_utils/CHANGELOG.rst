^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix integration tests (`#136 <https://github.com/ros-industrial/ros2_canopen/issues/136>`_)
* Update package versions to 0.1.0 (`#133 <https://github.com/ros-industrial/ros2_canopen/issues/133>`_)
* Beta release preparations (`#120 <https://github.com/ros-industrial/ros2_canopen/issues/120>`_)
  * Improve lely compilation time
  * Bump lely_core_librries to version 2.3.2
  * Add license files
  * Adapt package xml
  * Add changelogs - forthcoming for now.
  * Update readme
  * Add apacje-2.0 license notifications to files
  ---------
* Include rpdo/tpdo test in launch_test. (`#98 <https://github.com/ros-industrial/ros2_canopen/issues/98>`_)
  * Implement rpdo/tpdo test
  * Removed unnecessary files
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Add lifecycle to service-based operation (`#34 <https://github.com/ros-industrial/ros2_canopen/issues/34>`_)
  * Add check if remote object already exists to avoid multiple objects with same target.
  * Renaming and changes to MasterNode
  * restrucutring for lifecycle support
  * changes to build
  * Add lifecycle to drivers, masters and add device manager
  * Add lifecycled operation canopen_core
  * Added non lifecycle stuff to canopen_core
  * Add lifecyle to canopen_base_driver
  * Add lifecycle to canopen_proxy_driver
  * restructured canopen_core for lifecycle support
  * restructured canopen_base_driver for lifecycle support
  * Restrucutured canopen_proxy_driver for lifecycle support
  * Restructured canopen_402_driver for lifecycle support
  * Add canopen_mock_slave add cia402 slave
  * add canopen_tests package for testing canopen stack
  * Disable linting for the moment and some foxy compat changes
  * Further changes for foxy compatability
* Configuration manager integration (`#14 <https://github.com/ros-industrial/ros2_canopen/issues/14>`_)
  * Add longer startup delay and test documentation
  * Add speed and position publisher
  * Create Configuration Manager
  * make MasterNode a component and add configuration manager functionalities
  * add configuration manager functionalities
  * add configuration manger functionalities
  * Add documentation for Configuration Manager
  * add info messages and documentation
  * update launch files and configuration fiels
  * add can_utils package
  * add info text
  * simplify dependencies
  * remove tests from can_utils
  * avoid tests for canopen_utils
  * changes info logging and adds nmt and sdo tests
  * add tests
  * remove launch_tests from cmake
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Vishnuprasad Prachandabhanu
