import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_mock_slave"), "launch"
                ),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={"node_id": "2", "node_name": "slave_node_1"}.items(),
    )

    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("canopen_mock_slave"), "launch"
                ),
                "/basic_slave.launch.py",
            ]
        ),
        launch_arguments={"node_id": "3", "node_name": "slave_node_2"}.items(),
    )

    print(os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "proxy_write_sdo",
                "master.dcf",
            ))
                
    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "proxy_write_sdo",
                "master.dcf",
            ),
            "master_bin": "",
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "proxy_write_sdo.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    return LaunchDescription(
            [slave_node_1, slave_node_2, device_container]
        )
