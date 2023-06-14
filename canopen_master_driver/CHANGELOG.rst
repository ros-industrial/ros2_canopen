^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_master_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix a number of build warnings (`#137 <https://github.com/ros-industrial/ros2_canopen/issues/137>`_)
  * Fix a number of build warnings
  * Try rclcpp::QoS
  * Set QoS in device_container
  ---------
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
* Remove type indication from msg and srv interfaces (`#112 <https://github.com/ros-industrial/ros2_canopen/issues/112>`_)
  * Get slave eds and bin in node_canopen_driver
  * Add dictionary to base driver
  * Enable dictionary in proxy drivers
  * Add a few test objects
  * Add pdo checks
  * Adjust 402 driver
  * Fix tests
  * rename to get_xx_queue
  * Add typed sdo operations
  * Remove object datatype where possible
  ---------
* Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
* Merge branch 'master' into patch-2
* Merge remote-tracking branch 'ros/master'
* Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
* Merge pull request `#60 <https://github.com/ros-industrial/ros2_canopen/issues/60>`_ from ipa-cmh/merge-non-lifecycle-and-lifecycle-drivers
  Streamline driver and master infrastructure
* undo renaming can_interface_name -> can_interface
* Streamline logging
* Fix canopen_master_driver tests
* Fix canopen_master_driver for explicit instantiation
* Feature parity for lifecycle nodes
* Add master dcfs and remove from gitignore
* Add device container and general changes to make things work.
* Add canopen_master_driver package and contents
* Contributors: Błażej Sowa, Christoph Hellmann Santos
