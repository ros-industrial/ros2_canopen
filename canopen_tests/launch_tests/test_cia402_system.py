#    Copyright 2026 ROS-Industrial
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import os
import subprocess
import time
import unittest

import launch_testing
import pytest
import rclpy
from ament_index_python import get_package_share_directory
from controller_manager_msgs.srv import ListHardwareComponents, SetHardwareComponentState
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State

HARDWARE = "canopen_test_system"


@pytest.mark.rostest
def generate_test_description():
    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("canopen_tests"), "launch", "cia402_system.launch.py"
            )
        )
    )
    return (LaunchDescription([launch_desc, launch_testing.actions.ReadyToTest()]), {})


class TestHeartbeatTimeout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_heartbeat_timeout")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def call(self, srv_type, name, request):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), name)
        return future.result()

    def hardware_state(self):
        res = self.call(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
            ListHardwareComponents.Request(),
        )
        return next(c.state.id for c in res.component if c.name == HARDWARE)

    def wait_for(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.2)
        return False

    def activate(self):
        req = SetHardwareComponentState.Request(
            name=HARDWARE, target_state=State(id=State.PRIMARY_STATE_ACTIVE, label="active")
        )
        return self.call(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state", req
        ).ok

    def pause_slave(self, paused):
        # A stopped process sends no heartbeat and does not reboot, like a pulled cable.
        signal = "-STOP" if paused else "-CONT"
        subprocess.run(["pkill", signal, "-f", "cia402_slave_node"], check=True)

    def test_deactivate_on_heartbeat_timeout(self):
        active = State.PRIMARY_STATE_ACTIVE
        self.assertTrue(self.wait_for(lambda: self.hardware_state() == active, 30.0))

        self.pause_slave(True)
        try:
            inactive = State.PRIMARY_STATE_INACTIVE
            self.assertTrue(self.wait_for(lambda: self.hardware_state() == inactive, 10.0))
            self.assertFalse(self.activate())
        finally:
            self.pause_slave(False)

        self.assertTrue(self.wait_for(self.activate, 10.0))
        self.assertEqual(self.hardware_state(), active)
