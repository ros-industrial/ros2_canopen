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

"""Cia402System against a fake CiA 402 slave, driven through ros2_control controllers."""

import math
import os

import launch
import launch_testing
import pytest
from ament_index_python import get_package_share_directory
from canopen_utils.launch_test_node import LaunchTestCase
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

JOINT = "node_1"
CONTROLLER = "/cia402_device_1_controller"


@pytest.mark.rostest
def generate_test_description():
    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("canopen_tests"), "launch", "cia402_system.launch.py"
            )
        )
    )
    ready_to_test = launch.actions.TimerAction(
        period=5.0, actions=[launch_testing.actions.ReadyToTest()]
    )
    return LaunchDescription([launch_desc, ready_to_test]), {}


# test methods run in name order and build on each other
class TestCia402System(LaunchTestCase):

    def trigger(self, service):
        response = self.node.call(f"{CONTROLLER}/{service}", Trigger, Trigger.Request(), 10.0)
        self.assertTrue(response.success, f"{service} failed: {response.message}")

    def move_to(self, target):
        publisher = self.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        try:
            position = self.node.joint_position(
                JOINT,
                timeout=20.0,
                predicate=lambda p: abs(p - target) < 0.01,
                on_tick=lambda: publisher.publish(Float64MultiArray(data=[target])),
            )
        finally:
            self.node.destroy_publisher(publisher)
        self.assertIsNotNone(position, f"{JOINT} did not reach {target}")

    def test_1_controllers_active(self):
        self.node.wait_for_active_controllers(
            [
                "joint_state_broadcaster",
                "cia402_device_1_controller",
                "forward_position_controller",
            ]
        )

    def test_2_states_published(self):
        position = self.node.joint_position(JOINT, predicate=math.isfinite)
        self.assertIsNotNone(position, "no finite position state published")

    def test_3_init(self):
        self.trigger("init")

    def test_4_position_mode(self):
        self.trigger("position_mode")

    def test_5_position_command_moves_motor(self):
        self.move_to(1.0)
        self.move_to(-0.5)

    def test_6_halt_and_recover(self):
        self.trigger("halt")
        self.trigger("recover")
