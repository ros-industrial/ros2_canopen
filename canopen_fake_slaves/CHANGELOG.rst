^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package canopen_fake_slaves
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.9 (2024-04-16)
------------------

0.3.0 (2024-12-12)
------------------
* fix loop timer for run_velocity_mode
* Fix clang format
* Add comments for the fake slave function
* Fix the thread issue.
* Apply suggestions from code review
  Co-authored-by: Dr. Denis <denis@stoglrobotics.de>
* Working version.
* Periodic messages sent, but not received properly.
* Working logic. Still have to work on the edf file.
* Put the periodic messages in OnWrite
* Extend fake_slaves to publish messages via rpdo

0.2.12 (2024-04-22)
-------------------
* Merge pull request `#265 <https://github.com/ros-industrial/ros2_canopen/issues/265>`_ from kurtist123/feature/expose-fake-slave-includes
  build: export include directories
* build: export include directories
* 0.2.9
* forthcoming
* Contributors: Kurtis Thrush, Vishnuprasad Prachandabhanu, ipa-vsp

0.2.8 (2024-01-19)
------------------
* Add fake profile velocity (`#230 <https://github.com/ros-industrial/ros2_canopen/issues/230>`_)
  * Add simple sequence homing emulation
  * Add fake velocity mode
  * Formatting
  ---------
* Add simple sequence homing emulation (`#229 <https://github.com/ros-industrial/ros2_canopen/issues/229>`_)
* Contributors: Christoph Hellmann Santos

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

0.2.2 (2023-06-21)
------------------

0.2.1 (2023-06-21)
------------------
* Fix fake slave for PDOs
* Contributors: Christoph Hellmann Santos

0.2.0 (2023-06-14)
------------------
* Created package
* Contributors: Błażej Sowa, Christoph Hellmann Santos, James Ward, Vishnuprasad Prachandabhanu
