^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
