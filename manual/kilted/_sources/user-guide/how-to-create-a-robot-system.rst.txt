How to create a robot system with ros2_control
==============================================
This guide describes how to create a simple robot using the robot system hardware interface
leveragin ros2_control.

1. Create a new configuration package with the name ``canopen_robot_control_example``.

  .. code-block:: console

    $ ros2 pkg create --dependencies canopen lely_core_libraries --build-type ament_cmake {package_name}
    $ cd {package_name}
    $ rm -rf src
    $ rm -rf include
    $ mkdir -p launch
    $ mkdir -p config

2. Create a new bus configuration folder with the name ``robot_control``.

  .. code-block:: console

    $ mkdir -p config/robot_control

3. Create a new bus configuration file with the name ``bus.yml``.

  .. code-block:: console

    $ touch config/robot_control/bus.yml

4. Add the following content to the ``bus.yml`` file.

.. code-block:: yaml

  options:
    dcf_path: "@BUS_CONFIG_PATH@"

  master:
    node_id: 1
    driver: "ros2_canopen::MasterDriver"
    package: "canopen_master_driver"
    sync_period: 10000

  defaults:
    dcf: "cia402_slave.eds"
    driver: "ros2_canopen::Cia402Driver"
    package: "canopen_402_driver"
    period: 10
    position_mode: 1
    revision_number: 0
    sdo:
      - {index: 0x60C2, sub_index: 1, value: 50} # Set interpolation time for cyclic modes to 50 ms
      - {index: 0x60C2, sub_index: 2, value: -3} # Set base 10-3s
      - {index: 0x6081, sub_index: 0, value: 1000}
      - {index: 0x6083, sub_index: 0, value: 2000}
      - {index: 0x6060, sub_index: 0, value: 7}
    tpdo: # TPDO needed statusword, actual velocity, actual position, mode of operation
      1:
        enabled: true
        cob_id: "auto"
        transmission: 0x01
        mapping:
          - {index: 0x6041, sub_index: 0} # status word
          - {index: 0x6061, sub_index: 0} # mode of operation display
      2:
        enabled: true
        cob_id: "auto"
        transmission: 0x01
        mapping:
          - {index: 0x6064, sub_index: 0} # position actual value
          - {index: 0x606c, sub_index: 0} # velocity actual position
    rpdo: # RPDO needed controlword, target position, target velocity, mode of operation
      1:
        enabled: true
        cob_id: "auto"
        mapping:
        - {index: 0x6040, sub_index: 0} # controlword
        - {index: 0x6060, sub_index: 0} # mode of operation
      2:
        enabled: true
        cob_id: "auto"
        mapping:
        - {index: 0x607A, sub_index: 0} # target position

  nodes:
    joint_1:
      node_id: 2
    joint_2:
      node_id: 3

5. Copy the ``cia402_slave.eds`` file from the ``canopen_tests/config/robot_control`` package to the ``config/robot_control`` folder.

6. Create a ros2_controllers.yaml and add the following content.

.. code-block:: yaml

  controller_manager:
    ros__parameters:
      update_rate: 100  # Hz
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      forward_position_controller:
        type: forward_command_controller/ForwardCommandController

  forward_position_controller:
    ros__parameters:
      joints:
        - joint1
        - joint2
      interface_name: position

7. Create a launch file with the name ``robot_control.launch.py`` in the launch directory of your package and add the following content.

.. code-block:: python

  from launch import LaunchDescription
  from launch.actions import DeclareLaunchArgument
  from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
  from launch_ros.actions import Node
  from launch_ros.substitutions import FindPackageShare
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource


  def generate_launch_description():
      robot_description_content = Command(
          [
              PathJoinSubstitution([FindExecutable(name="xacro")]),
              " ",
              PathJoinSubstitution(
                  [
                      FindPackageShare("canopen_tests"),
                      "urdf",
                      "robot_controller",
                      "robot_controller.urdf.xacro",
                  ]
              ),
          ]
      )
      robot_description = {"robot_description": robot_description_content}
      robot_control_config = PathJoinSubstitution(
          [FindPackageShare("canopen_tests"), "config/robot_control", "ros2_controllers.yaml"]
      )

      control_node = Node(
          package="controller_manager",
          executable="ros2_control_node",
          parameters=[robot_description, robot_control_config],
          output="screen",
      )

      joint_state_broadcaster_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
      )

      forward_position_controller_spawner = Node(
          package="controller_manager",
          executable="spawner",
          arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
      )

      robot_state_publisher_node = Node(
          package="robot_state_publisher",
          executable="robot_state_publisher",
          output="both",
          parameters=[robot_description],
      )

      slave_config = PathJoinSubstitution(
          [FindPackageShare("canopen_tests"), "config/robot_control", "cia402_slave.eds"]
      )

      slave_launch = PathJoinSubstitution(
          [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
      )
      slave_node_1 = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(slave_launch),
          launch_arguments={
              "node_id": "2",
              "node_name": "slave_node_1",
              "slave_config": slave_config,
          }.items(),
      )

      slave_node_2 = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(slave_launch),
          launch_arguments={
              "node_id": "3",
              "node_name": "slave_node_2",
              "slave_config": slave_config,
          }.items(),
      )

      nodes_to_start = [
          control_node,
          joint_state_broadcaster_spawner,
          forward_position_controller_spawner,
          robot_state_publisher_node,
          slave_node_1,
          slave_node_2
      ]

      return LaunchDescription(nodes_to_start)

8. Create a urdf folder add all files from the ``canopen_tests/urdf/robot_controller`` package to the urdf folder of your package.

9. Edit the CMakeLists.txt file of your package and add the following lines after the find_package section.

.. code-block:: cmake

  cogen_dcf(robot_control)

  install(DIRECTORY
  launch urdf
  DESTINATION share/${PROJECT_NAME})

10. Build your package and source the setup.bash file.
11. Start your launch file
12. You can now control the robot with the forward_command_controller. You can as well visualize the robot in
    rviz by adding a tf or a robot model and setting the fixed frame to ``base_link``. You can move the robot with the
    following command.

.. code-block:: bash

  ros2 topic pub /joint1/forward_position_controller/command std_msgs/msg/Float64 "data: [1.0, 1.0]"
