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
import time
import pytest
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import threading
import rclpy
from rclpy.node import Node
from canopen_utils.launch_test_node import LaunchTestNode
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID, COTargetDouble
from canopen_interfaces.msg import COData
from std_srvs.srv import Trigger
import unittest


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/cia402_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestSDO(unittest.TestCase):
    def run_node(self):
        while self.ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def setUp(cls):
        cls.ok = True
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.thread = threading.Thread(target=cls.run_node, args=[cls])
        cls.thread.start()

    @classmethod
    def tearDown(cls):
        cls.ok = False
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_sdo_read(self):
        req = CORead.Request()
        req.index = 0x1000
        req.subindex = 0

        res = CORead.Response()
        res.success = True
        res.data = 0xFFFF0192

        self.node.call_service("cia402_device_1/sdo_read", CORead, req, res)

    def test_init(self):
        req = Trigger.Request()

        res = Trigger.Response()
        res.success = True

        self.node.call_service("cia402_device_1/init", Trigger, req, res)

    def test_position_mode(self):
        req = Trigger.Request()

        res = Trigger.Response()
        res.success = True

        self.node.call_service("cia402_device_1/init", Trigger, req, res)
        self.node.call_service("cia402_device_1/position_mode", Trigger, req, res)

        target_req = COTargetDouble.Request()
        target_req.target = 1.0
        target_res = COTargetDouble.Response()
        target_res.success = True

        self.node.call_service("cia402_device_1/target", COTargetDouble, target_req, target_res)
