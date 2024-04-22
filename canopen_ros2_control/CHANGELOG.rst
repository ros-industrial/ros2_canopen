^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.11 (2024-04-22)
-------------------

0.2.10 (2024-04-22)
-------------------
* Merge pull request `#281 <https://github.com/ros-industrial/ros2_canopen/issues/281>`_ from ipa-vsp/humble
  Sync Humble
* Merge remote-tracking branch 'industrial/humble' into humble
* synch humble
* Fixed incorrect use of log level in cia402_data.hpp (`#213 <https://github.com/ros-industrial/ros2_canopen/issues/213>`_)
  This is in reference to issue `#179 <https://github.com/ros-industrial/ros2_canopen/issues/179>`_
* 0.2.7
* Update changelog
* [ros2controllers] Correct Proxy controller after changes and update tests. (`#148 <https://github.com/ros-industrial/ros2_canopen/issues/148>`_)
  * Correct Proxy controller after changes and update docs
  * Update codespellignore
  * Fix tests
  * Fix owns/ons error in tests
  * Fixup tests.
  ---------
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* 0.2.6
* Update changelogs
* 0.2.5
* Update changelogs
* 0.2.4
* Update changelog
* 0.2.3
* Update Changelogs
* Update Changelogs
* Update Changelogs
* Solve buildfarm issues (`#155 <https://github.com/ros-industrial/ros2_canopen/issues/155>`_)
  * Move exec deps from ros2_control to tests
  * Remove mkdir in install dir from cogen and dcfgen
  This causes a permission denied error on buildfarm.
  The install command creates it anyways
  ---------
* 0.2.2
* Update Changelogs
* 0.2.7
* Update changelog
* [ros2controllers] Correct Proxy controller after changes and update tests. (`#148 <https://github.com/ros-industrial/ros2_canopen/issues/148>`_)
  * Correct Proxy controller after changes and update docs
  * Update codespellignore
  * Fix tests
  * Fix owns/ons error in tests
  * Fixup tests.
  ---------
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* 0.2.6
* Update changelogs
* 0.2.5
* Update changelogs
* 0.2.4
* Update changelog
* 0.2.3
* Update Changelogs
* Update Changelogs
* Update Changelogs
* Solve buildfarm issues (`#155 <https://github.com/ros-industrial/ros2_canopen/issues/155>`_)
  * Move exec deps from ros2_control to tests
  * Remove mkdir in install dir from cogen and dcfgen
  This causes a permission denied error on buildfarm.
  The install command creates it anyways
  ---------
* 0.2.2
* Update Changelogs
* Sync humble branch with 0.2.1 release  (`#153 <https://github.com/ros-industrial/ros2_canopen/issues/153>`_)
  * Update rolling.yml
  * Merge pull request `#152 <https://github.com/ros-industrial/ros2_canopen/issues/152>`_ from ros-industrial/fix-bdist-dep-in-lely-core
  Do not build dcf-tools bdist in lely_core_libraries
  * Merge pull request `#142 <https://github.com/ros-industrial/ros2_canopen/issues/142>`_ from ros-industrial/iron-test-fixes-controllers
  Don't use ros2_control_test_assets since not needed anymore. Add load tests for all controllers.
  * Merge pull request `#145 <https://github.com/ros-industrial/ros2_canopen/issues/145>`_ from ros-industrial/consistent-output-of-hex-values
  Use consistenlty (uppercase) HEX output for NodeID and Index.
  * Merge pull request `#151 <https://github.com/ros-industrial/ros2_canopen/issues/151>`_ from ros-industrial/upgrade-launch-tests
  Fix proxy driver lifecyle and launch tests
  * Update Changelogs
  * 0.2.1
  * Fix build warnings
  * Update status badge.
  ---------
  Co-authored-by: Christoph Hellmann Santos <51296695+ipa-cmh@users.noreply.github.com>
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
* 0.2.1
* Update Changelogs
* Merge pull request `#145 <https://github.com/ros-industrial/ros2_canopen/issues/145>`_ from ros-industrial/consistent-output-of-hex-values
  Use consistenlty (uppercase) HEX output for NodeID and Index.
* Merge pull request `#144 <https://github.com/ros-industrial/ros2_canopen/issues/144>`_ from ipa-vsp/humble
  Sync humble with Beta (0.2.0) release
* 0.2.0
* Update changelog
* Update changelofs
* Merge branch 'StoglRobotics-forks-use-can-interface-name-consistently'
* Bring ros2_control test launches to canopen_tests pkg (`#131 <https://github.com/ros-industrial/ros2_canopen/issues/131>`_)
* Update package versions to 0.1.0 (`#133 <https://github.com/ros-industrial/ros2_canopen/issues/133>`_)
* Fix interpolated mode switch in CiA402 (`#124 <https://github.com/ros-industrial/ros2_canopen/issues/124>`_)
* Beta release preparations (`#120 <https://github.com/ros-industrial/ros2_canopen/issues/120>`_)
  * Improve lely compilation time
  * Bump lely_core_librries to version 2.3.2
  * Add license files
  * Adapt package xml
  * Add changelogs - forthcoming for now.
  * Update readme
  * Add apacje-2.0 license notifications to files
  ---------
* Sync humble with newest changes (`#123 <https://github.com/ros-industrial/ros2_canopen/issues/123>`_)
  * Introduce canopen system interface.
  * Enable easy testing temporarily.
  * Print config paths on init.
  * Add device manager and executor.
  * Start device manager in system interface.
  * Add nmt and rpdo callbacks.
  * Remove pedantic cmake flags.
  * Add internal caching structures for canopen nodes.
  * Expose necessary stuff from proxy driver.
  * Apply suggestions from code review
  * Fix dependencies for canopen_ros2_control.
  * Move device manager instantation into on_config.
  * Introduce canopen system interface.
  * Enable easy testing temporarily.
  * Print config paths on init.
  * Add device manager and executor.
  * Start device manager in system interface.
  * Add nmt and rpdo callbacks.
  * Remove pedantic cmake flags.
  * Add internal caching structures for canopen nodes.
  * Expose necessary stuff from proxy driver.
  * Apply suggestions from code review
  * Add missig dependencies and execute tests only when testing.
  * Remove unneccesary deps.
  * Rename "device manager" to "device container" and disable test because it is now working in the current setup.
  * Update .gitignore
  * Remove some unecessary changes.
  * Fix merging issues.
  * Update visibility-control macros.
  * Disable test because they can not be eaisly tested.
  * Update canopen_core/CMakeLists.txt
  * Solve Boot Error (`#49 <https://github.com/ros-industrial/ros2_canopen/issues/49>`_)
  * Update CI for new version branches
  * Simplify configuration folder and use existing .eds, .dcf file. Improve test launch file. Update runtime deps.
  * Add template for canopen proxy controller.
  * Add dummy services, rt publishers and subscribers to proxy controller.
  * Expose controller plugin. Start canopen proxy controller instance in example.
  * Add publishing of rpdo and nmt state.
  * Add service one shot mechanisms.
  * Apply suggestions from code review
  * Add tests to canopen_tests
  * Reorganise test launch system
  * update package.xml
  * further changes
  * Remove bus.yml
  * Adapt canopen_system.launch.py for 2 nodes
  * Update documentation in the readme
  * Add documentation about testing ros2_control generic interface.
  * Disable dynamic loading for containers (`#50 <https://github.com/ros-industrial/ros2_canopen/issues/50>`_)
  * disable loader service
  * Remove artifacts
  * Publish joint state instead of velocity topics (`#47 <https://github.com/ros-industrial/ros2_canopen/issues/47>`_)
  * disable loader service
  * add custom target/command and install to macro
  * publish jointstate
  * correct variable name squiggle
  * Minor changes to driver and slave
  * Update lely core library
  * Add sensor_msgs to dependencies
  * Remove artifacts
  * Remove some artifacts
  * Add configuration parameter passthrough (`#52 <https://github.com/ros-industrial/ros2_canopen/issues/52>`_)
  * Apply suggestions from code review
  Co-authored-by: Christoph Hellmann Santos <51296695+ipa-cmh@users.noreply.github.com>
  * Correct macro names.
  * Add service qos specific profile.
  * Introduce tests. Adapt proxy controller for easier testing.
  * Update README.md
  * make documentation on test with ros2_control more detailed
  Make the test documentation a small example with explanations of the functionality.
  * Add in code documentation for canopen_core (`#53 <https://github.com/ros-industrial/ros2_canopen/issues/53>`_)
  * Document device container node
  * Document lely_master_bridge
  * Document Lifecycle Device Container
  * Document Lifecycle Device Manager
  * Document LifecyleMasterNode
  * Document Master Node
  * Fix error
  * Document lifecycle base driver
  * Document lely bridge
  * Document canopen_proxy_driver
  * Document canopen_402_driver
  * Restructure documentation (`#37 <https://github.com/ros-industrial/ros2_canopen/issues/37>`_)
  * Document device container node
  * Document lely_master_bridge
  * Document Lifecycle Device Container
  * Document Lifecycle Device Manager
  * Document LifecyleMasterNode
  * Document Master Node
  * Fix error
  * Document lifecycle base driver
  * Document lely bridge
  * Document canopen_proxy_driver
  * Document canopen_402_driver
  * Update sphinx documentation
  * Add profiled position to mock slave (`#58 <https://github.com/ros-industrial/ros2_canopen/issues/58>`_)
  * Implement review requests regarding tests.
  * Add core node interfaces
  * Add errors
  * Add node base classes
  * Remove device and do some renaming
  * Add tests to canopen core
  * Update CmakeFile of canopen core
  * Add canopen_master_driver package and contents
  * Make changes to canopen_base_driver for new structure
  * Change canopen_base_driver for templating problems
  * Add canopen_proxy_driver with new framework
  * canopen_402_driver adaption to new framework
  * Update header guards
  * Add device container and general changes to make things work.
  * Add function to device container
  * Integration with ros2_control
  * Add type accessor functions to device_container
  * add node interface accessor  and lifecycle information to drivers
  * Add master dcfs and remove from gitignore
  * Add 402 driver functions for ros2_control
  * Add CanopenDriverInterface Documentation
  * Feature parity for lifecycle nodes
  * Fix canopen_master_driver for explicit instantiation
  * Fix canopen_master_driver tests
  * Fix tests canopen_core
  * Fix tests base driver
  * Try running source install/setup.bash
  * Fix integration tests
  * Integrate lifecycle manager
  * Fix get speed and get position
  * Fix node_canopen_402_drivers add_to_master and handlers
  * Streamline logging
  * Remove canopen_lifecycle.launch.py as it i no longer needed.
  * Add lifecycle manager to device_container library
  * Undo formatting in ros2_control
  * fix ci
  * Fix 402 issues from testing
  * undo renaming can_interface_name -> can_interface
  * Publish to joint_states, not joint_state (`#63 <https://github.com/ros-industrial/ros2_canopen/issues/63>`_)
  Co-authored-by: G.A. vd. Hoorn <g.a.vanderhoorn@tudelft.nl>
  Co-authored-by: Christoph Hellmann Santos <christoph.hellmann.santos@ipa.fraunhofer.de>
  * Add unit tests for canopen_core (`#64 <https://github.com/ros-industrial/ros2_canopen/issues/64>`_)
  * Testing changes to canopen_core
  * Testing changes to canopen_base_driver and canopen_402_driver
  * Add canopen_core tests (90% coverage)
  * Fix DriverException error in canopen_402_driver
  * Catch errors in nmt and rpdo listeners
  * Fix naming issues
  * Fix deactivate transition
  * Fix unclean shutdown
  * Rename canopen_mock_slave package to canopen_fake_slaves (`#66 <https://github.com/ros-industrial/ros2_canopen/issues/66>`_)
  * Testing changes to canopen_core
  * Testing changes to canopen_base_driver and canopen_402_driver
  * Add canopen_core tests (90% coverage)
  * Fix DriverException error in canopen_402_driver
  * Catch errors in nmt and rpdo listeners
  * Fix naming issues
  * Fix deactivate transition
  * Fix unclean shutdown
  * Rename canopen_mock_slave to canopen_fake_slaves
  * Build flage CANOPEN_ENABLED for disabling tests on CI.
  * Update deployment
  * Documentation for streamlined design (`#67 <https://github.com/ros-industrial/ros2_canopen/issues/67>`_)
  * Add canopen_core tests (90% coverage)
  * Restructure and add plantuml
  * Changes to quickstart/configuration
  * Revert "Add canopen_core tests (90% coverage)" as it is not needed.
  This reverts commit 771c498347f190777fb28edfd5044b96cbfd7bf0.
  * Create custom driver documentation
  * Remove breathe api reference and use doxygen
  * Update interface and naming information for drivers
  * Update  test documentation
  * install plantuml
  * Update README.md
  * Add bare-bone 402 profile system interface.
  * Add position and speed getter.
  * State and command interfaces.
  * Update dependencies.
  * To protected members for easier inheritance policy.
  * Fix public fcn visibility.
  * Adapt 402 hardware interface to device container getter.
  * Prepare read/write/
  * Extend 402 functions via public methods - same as callback based ones.
  * Expose 402 main functionalities to ros2_control system interface.
  * Add vel and pos interfaves.
  * Update proxy canopen system.
  * Add basic read and write. Divide targets into position, velocity, effort interfaces.
  * Duplicate some code for configure, init, write phase from proxy driver.
  * Set target based on condition.
  * Handle init, recover, halt. Switch modes.
  * Fix feedback for services for proxy driver and controlller.
  * Prepare cia 402 device controller.
  * Add base function ret values first.
  * State and command interfaces.
  * Add services for one shot interfaces in cia402 profile.
  * Better handling of base class on_methods.
  * Update runtime deps.
  * Fix joint states scaling.
  * Add virtual can example for cia 402.
  * Fix internal launch test.
  * Fix proxy test.
  * intra_process_comms
  * Doxygen documentation for canopen_core (`#78 <https://github.com/ros-industrial/ros2_canopen/issues/78>`_)
  * canopen_core in code documentation
  * Some more documentation
  * intra_process_comms
  * Doxygen documentation for canopen_core (`#78 <https://github.com/ros-industrial/ros2_canopen/issues/78>`_)
  * canopen_core in code documentation
  * Some more documentation
  * Remove scalers
  * Clean cia402 fake shutdown (`#72 <https://github.com/ros-industrial/ros2_canopen/issues/72>`_)
  * adapt fake cia402 slave
  * Handle demand set master failure (`#70 <https://github.com/ros-industrial/ros2_canopen/issues/70>`_)
  * adapt fake cia402 slave
  * Add retries for demand_set_master in case of failure
  * Scaling factors for position and velocity (`#74 <https://github.com/ros-industrial/ros2_canopen/issues/74>`_)
  * Introduce scaling factors
  * Remove false license statements (`#76 <https://github.com/ros-industrial/ros2_canopen/issues/76>`_)
  * Remove false license statements
  * Disable device container tests (`#77 <https://github.com/ros-industrial/ros2_canopen/issues/77>`_)
  * Add formatters as used in ros2_control framework.
  * Changes to format and checkers
  * Substitute @BUS_CONFIG_PATH@ in bus configuration file
  * Use @BUS_CONFIG_PATH@ variable in bus configuration files
  * Precommit changes (`#79 <https://github.com/ros-industrial/ros2_canopen/issues/79>`_)
  * Precommit changes
  * Update to clang-format-14
  * Don't treat options section as another device
  * Use options section in test bus config files
  * Remove references to sympy.true (`#84 <https://github.com/ros-industrial/ros2_canopen/issues/84>`_)
  Co-authored-by: James Ward <j.ward@sydney.edu.au>
  * add short documentation
  * Add dcf_path to bus.ymls
  * Don't treat options as driver
  * Format updates
  * Better organize dependencies (`#88 <https://github.com/ros-industrial/ros2_canopen/issues/88>`_)
  * Add EMCY callback to base driver (`#91 <https://github.com/ros-industrial/ros2_canopen/issues/91>`_)
  * Add post build testing in readme (`#87 <https://github.com/ros-industrial/ros2_canopen/issues/87>`_)
  * Simplify 402 driver (`#89 <https://github.com/ros-industrial/ros2_canopen/issues/89>`_)
  * Split motor.hpp and motor.cpp into different files
  * Fix missing symbol error
  ---------
  * Add Interpolated Position Mode (linear only, no PT or PVT) (`#90 <https://github.com/ros-industrial/ros2_canopen/issues/90>`_)
  * Add Interpolated Position Mode (linear only, no PT or PVT)
  * add interpolated position mode to system interface
  * Add interpolated position mode to controllers.
  * Add to interpolated position mode to documentation
  ---------
  * Fix typo in README (`#92 <https://github.com/ros-industrial/ros2_canopen/issues/92>`_)
  * Correct repo link (`#94 <https://github.com/ros-industrial/ros2_canopen/issues/94>`_)
  * Implemented thread-safe queue for rpdo and emcy listener (`#97 <https://github.com/ros-industrial/ros2_canopen/issues/97>`_)
  * Boost lock free queue implemetation
  * include boost libraries in CMakelists
  * Testing rpdo/tpdo ping pond
  * pre-commit changes
  * Bugfix: implemented timeout for wait_and_pop to avoid thread blocking
  * Fixed typo
  * pre-commit update
  * FIxed: properly export Boost libraries
  * Update code documentation
  * proper vel and pos scaling from device
  * Include rpdo/tpdo test in launch_test. (`#98 <https://github.com/ros-industrial/ros2_canopen/issues/98>`_)
  * Implement rpdo/tpdo test
  * Removed unnecessary files
  * Fix stack smashing (`#103 <https://github.com/ros-industrial/ros2_canopen/issues/103>`_)
  * Motor Profile Updates (`#101 <https://github.com/ros-industrial/ros2_canopen/issues/101>`_)
  * Extend and fix info statement.
  * Fix service handler overwriting.
  * Consider enum 3 as profiled velocity. Remove some code duplication by reusing private setters in service cbs. Create setter for interpolated position mode.
  * Fix cyclic position mode.
  * Simplify write method cases defined by mode of op.
  * Add driver dictionaries (`#110 <https://github.com/ros-industrial/ros2_canopen/issues/110>`_)
  * Get slave eds and bin in node_canopen_driver
  * Add dictionary to base driver
  * Enable dictionary in proxy drivers
  * Add a few test objects
  * Add pdo checks
  * Adjust 402 driver
  * Fix tests
  * rename to get_xx_queue
  * Add typed sdo operations
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
  * Reduce processor load (`#111 <https://github.com/ros-industrial/ros2_canopen/issues/111>`_)
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
  * Add plain operation mode setting + switchingstate
  * Add robot system interface
  * Add robot system controller
  * Add robot_system_tests
  * Add a bit of documentation
  * Add in code documentation
  * Fix bug
  * Add examples section
  * Fix set_target for interpolated mode
  * Switch to rclcpp::sleep_for
  * Fix initialization for state and command interface variables
  * Add remade robot system interfce
  * Add copyright info
  * Fix missing return statement
  * processing behavior improvement
  * Minor changes to make things work
  * Add poll_timer_callback
  * Fix format
  * Add polling mode variable for config.
  ---------
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
  * Robot system interface (`#113 <https://github.com/ros-industrial/ros2_canopen/issues/113>`_)
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
  * Add plain operation mode setting + switchingstate
  * Add robot system interface
  * Add robot system controller
  * Add robot_system_tests
  * Add a bit of documentation
  * Add in code documentation
  * Fix bug
  * Add examples section
  * Fix set_target for interpolated mode
  * Switch to rclcpp::sleep_for
  * Fix initialization for state and command interface variables
  * Add remade robot system interfce
  * Add copyright info
  * Fix missing return statement
  * processing behavior improvement
  * Minor changes to make things work
  * Add poll_timer_callback
  * Fix format
  * Add polling mode variable for config.
  ---------
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
  * Enable simplified bus.yml format (`#115 <https://github.com/ros-industrial/ros2_canopen/issues/115>`_)
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
  * Add plain operation mode setting + switchingstate
  * Add robot system interface
  * Add robot system controller
  * Add robot_system_tests
  * Add a bit of documentation
  * Add in code documentation
  * Fix bug
  * Add examples section
  * Fix set_target for interpolated mode
  * Switch to rclcpp::sleep_for
  * Fix initialization for state and command interface variables
  * Add remade robot system interfce
  * Add copyright info
  * Fix missing return statement
  * processing behavior improvement
  * Minor changes to make things work
  * Add poll_timer_callback
  * Fix format
  * Add polling mode variable for config.
  * Add cogen
  * Add example usage for cogen
  * Remove explicit path
  ---------
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
  * add dedicated documentation for humble, rolling and iron
  ---------
  Co-authored-by: Lovro <lovro.ivanov@gmail.com>
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <denis@stoglrobotics.de>
  Co-authored-by: Dr.-Ing. Denis Štogl <denis.stogl@stoglrobotics.de>
  Co-authored-by: G.A. vd. Hoorn <g.a.vanderhoorn@tudelft.nl>
  Co-authored-by: Błażej Sowa <bsowa123@gmail.com>
  Co-authored-by: James Ward <james@robomo.co>
  Co-authored-by: James Ward <j.ward@sydney.edu.au>
  Co-authored-by: Chris Schindlbeck <chris.schindlbeck@gmail.com>
  Co-authored-by: Vishnuprasad Prachandabhanu <32260301+ipa-vsp@users.noreply.github.com>
  Co-authored-by: Vishnuprasad Prachandabhanu <vishnu.pbhat93@gmail.com>
* Contributors: Christoph Hellmann Santos, Dr. Denis, Jerome Justin, Vishnuprasad Prachandabhanu, ipa-vsp
