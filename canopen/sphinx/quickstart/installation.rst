Installation
===============================
- Clone ros2_canopen into your ROS2 workspace's source folder, install dependencies and build with colcon and your done.
- In order to create a workspace

.. code-block:: console

   $ mkdir -p ~/name_ws/src
   $ cd ~/name_ws/src

.. note::
    Change the ``name_ws`` to your desired workspace name.

.. code-block:: console

   $ git clone https://github.com/ros-industrial/ros2_canopen.git
   $ cd ..
   $ source /opt/ros/ros_distro/setup.bash # Replace the *ros distro as per your need.
   $ rosdep install --from-paths src/ros2_canopen --ignore-src -r -y
   $ colcon build
   $ source install/setup.bash

.. note::
   Kindly check the branch as per your configuration before cloning the repository.

Testing with pre-release binaries
---------------------------------
To test the package with pre-release binaries, you must first set up your ROS2 workspace to use the ROS2 pre-release repositories.
To do this, follow the instructions
`here <https://docs.ros.org/en/rolling/Installation/Testing.html#debian-testing-repository>`_ .
After that, you can install the packages using the following command,

.. code-block:: console

   $ sudo apt update
   $ sudo apt install ros-<distro>-canopen # Replace <distro> with your ROS2 distro
