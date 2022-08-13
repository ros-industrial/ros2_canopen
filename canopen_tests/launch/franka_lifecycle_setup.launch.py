import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    slave_eds_path = os.path.join(
                    get_package_share_directory("canopen_tests"), "config", "franka", "cia402_slave.eds"
                )

    for i in range(1,8):

        slave_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(
                        get_package_share_directory("canopen_mock_slave"), "launch"
                    ),
                    "/cia402_slave.launch.py",
                ]
            ),
            launch_arguments={
                "node_id": "{}".format(i+1), 
                "node_name": "panda_slave_{}".format(i),
                "slave_config": slave_eds_path,
                }.items(),
        )
        ld.add_action(slave_node)
                
    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen_lifecycle.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "franka",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "franka",
                "franka.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'panda_arm.urdf.xacro')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=false'])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
    rviz2 = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])


    state_publisher = Node(
        package="joint_state_publisher",
        name="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{
            "source_list": [
                "/panda_joint1/joint_state",
                "/panda_joint2/joint_state",
                "/panda_joint3/joint_state",
                "/panda_joint4/joint_state",
                "/panda_joint5/joint_state",
                "/panda_joint6/joint_state",
                "/panda_joint7/joint_state",
                ],
            "rate": 10
        }]
    )

    ld.add_action(device_container)
    ld.add_action(robot_state_publisher)
    ld.add_action(state_publisher)
    ld.add_action(rviz2)
    return ld
