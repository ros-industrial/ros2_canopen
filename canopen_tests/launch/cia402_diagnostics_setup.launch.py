import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
import launch_ros
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    slave_eds_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "config",
        "cia402_diagnostics",
        "cia402_slave.eds",
    )

    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "2",
            "node_name": "cia402_node_1",
            "slave_config": slave_eds_path,
        }.items(),
    )
    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "3",
            "node_name": "cia402_node_2",
            "slave_config": slave_eds_path,
        }.items(),
    )
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_fake_slaves"), "launch"),
                "/cia402_slave.launch.py",
            ]
        ),
        launch_arguments={
            "node_id": "4",
            "node_name": "cia402_node_4",
            "slave_config": slave_eds_path,
        }.items(),
    )
    master_bin_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "config",
        "cia402_diagnostics",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""
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
                "cia402_diagnostics",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia402_diagnostics",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    diagnostics_analyzer_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "launch",
        "analyzers",
        "cia402_diagnostic_analyzer.yaml",
    )

    diagnostics_aggregator_node = launch_ros.actions.Node(
        package="diagnostic_aggregator",
        executable="aggregator_node",
        output="screen",
        parameters=[diagnostics_analyzer_path],
    )

    return LaunchDescription(
        [slave_node_1, slave_node_2, slave_node_3, device_container, diagnostics_aggregator_node]
    )
