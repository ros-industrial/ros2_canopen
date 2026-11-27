#    Copyright 2022 Christoph Hellmann Santos
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
from time import sleep
import pytest
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from rclpy.node import Node
from canopen_utils.launch_test_node import LaunchTestCase
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID, COTargetDouble
from canopen_interfaces.msg import COData
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/robot_control_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestSDO(LaunchTestCase):
    def test_sdo_read(self):
        req = CORead.Request()
        req.index = 0x1000
        req.subindex = 0

        res = CORead.Response()
        res.success = True
        res.data = 0x60192

        self.node.call_service("joint_1/sdo_read", CORead, req, res)

    def test_forward_control(self):
        self.node.wait_for_active_controllers(
            ["joint_state_broadcaster", "forward_position_controller"]
        )
        publisher = self.node.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10
        )
        try:
            for target in ([1.0, 0.5], [-0.5, 0.0]):
                for joint, position in zip(["joint1", "joint2"], target):
                    reached = self.node.joint_position(
                        joint,
                        timeout=20.0,
                        predicate=lambda p, position=position: abs(p - position) < 0.01,
                        on_tick=lambda: publisher.publish(Float64MultiArray(data=target)),
                    )
                    self.assertIsNotNone(reached, f"{joint} did not reach {position}")
        finally:
            self.node.destroy_publisher(publisher)
