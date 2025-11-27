^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------

0.3.1 (2025-06-23)
------------------

0.3.0 (2024-12-12)
------------------
* Add new line to get checking pipeline okay
* Working version.
* Periodic messages sent, but not received properly.

0.2.12 (2024-04-22)
-------------------
* Merge pull request `#270 <https://github.com/ros-industrial/ros2_canopen/issues/270>`_ from gsalinas/can-namespace-pr
  Put components loaded by the device container into its namespace, if any.
* pre-commit update
* Add node on namespaces.
* Restore base version of canopen_system example.
* Restore cia402_system launch to original.
* Change cia402_system ros2_controllers back to original.
* Add comment about the use of ReplaceString.
* Push namespace to nodes in a group.
* Separate out example of how to use a namespaced ros2_control/cia402 based system.
* Remove commented-out arg and unneeded logging.
* Update canopen test system launch to use a namespace.
* Output robot_description_content to log during launch.
* WIP adding namespace to cia402_system launch.
* 0.2.9
* forthcoming
* Contributors: Gerry Salinas, Vishnuprasad Prachandabhanu, ipa-vsp

0.2.8 (2024-01-19)
------------------

0.2.7 (2023-06-30)
------------------
* Add missing license headers and activate ament_copyright
* Contributors: Christoph Hellmann Santos

0.2.6 (2023-06-24)
------------------

0.2.5 (2023-06-23)
------------------

0.2.4 (2023-06-22)
------------------

0.2.3 (2023-06-22)
------------------
* Update package.xml
* Solve buildfarm issues (`#155 <https://github.com/ros-industrial/ros2_canopen/issues/155>`_)
  * Move exec deps from ros2_control to tests
  * Remove mkdir in install dir from cogen and dcfgen
  This causes a permission denied error on buildfarm.
  The install command creates it anyways
  ---------
* Contributors: Christoph Hellmann Santos

0.2.2 (2023-06-21)
------------------

0.2.1 (2023-06-21)
------------------
* Add trvivial integration test for robot_control
* Add trivial integration tests for cia402_driver
* Use the more tidy launch_test_node
* Contributors: Christoph Hellmann Santos

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Błażej Sowa, Christoph Hellmann Santos, Denis Štogl, James Ward, Vishnuprasad Prachandabhanu
