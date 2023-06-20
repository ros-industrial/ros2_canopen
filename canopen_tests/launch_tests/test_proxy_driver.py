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
from canopen_interfaces.srv import CORead, COWrite, COReadID, COWriteID
from canopen_interfaces.msg import COData
from std_srvs.srv import Trigger
import unittest


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/proxy_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestNMT(unittest.TestCase):
    def run_node(self):
        while self.ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def setUpClass(cls):
        cls.ok = True
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.thread = threading.Thread(target=cls.run_node, args=[cls])
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.ok = False
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_reset_nmt(self):
        request = Trigger.Request()
        response = Trigger.Response()
        response.success = True
        self.node.call_service("proxy_device_2/nmt_reset_node", Trigger, request, response)
        time.sleep(1)


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
        req.index = 0x4000
        req.subindex = 0

        res = CORead.Response()
        res.success = True
        res.data = 0

        self.node.call_service("proxy_device_1/sdo_read", CORead, req, res)
        self.node.call_service("proxy_device_2/sdo_read", CORead, req, res)

    def test_sdo_write(self):
        index = 0x4000
        subindex = 0
        data = 100

        req = COWrite.Request()
        req.index = index
        req.subindex = subindex
        req.data = data

        res = COWrite.Response()
        res.success = True

        rreq = CORead.Request()
        rreq.index = index
        rreq.subindex = subindex

        rres = CORead.Response()
        rres.success = True
        rres.data = data

        self.node.call_service("proxy_device_1/sdo_write", COWrite, req, res)
        self.node.call_service("proxy_device_2/sdo_write", COWrite, req, res)
        time.sleep(0.01)
        self.node.call_service("proxy_device_1/sdo_read", CORead, rreq, rres)
        self.node.call_service("proxy_device_2/sdo_read", CORead, rreq, rres)
        req.data = 0
        time.sleep(0.01)
        self.node.call_service("proxy_device_1/sdo_write", COWrite, req, res)
        self.node.call_service("proxy_device_2/sdo_write", COWrite, req, res)
        time.sleep(0.01)


class TestSDOMaster(unittest.TestCase):
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
        req = COReadID.Request()
        req.index = 0x4000
        req.subindex = 0
        req.nodeid = 2
        req.canopen_datatype = 0x7

        res = COReadID.Response()
        res.success = True
        res.data = 0

        self.node.call_service("/master/sdo_read", COReadID, req, res)
        req.nodeid = 3
        self.node.call_service("/master/sdo_read", COReadID, req, res)

    def test_sdo_write(self):
        index = 0x4000
        subindex = 0
        data = 100
        req = COWriteID.Request()
        req.index = index
        req.subindex = subindex
        req.data = data
        req.canopen_datatype = 0x7
        req.nodeid = 2
        res = COWriteID.Response()
        res.success = True
        rreq = COReadID.Request()
        rreq.index = index
        rreq.subindex = subindex
        rreq.nodeid = 2
        rres = COReadID.Response()
        rres.success = True
        rres.data = data
        rreq.canopen_datatype = 0x7
        self.node.call_service("master/sdo_write", COWriteID, req, res)
        self.node.call_service("master/sdo_write", COWriteID, req, res)
        self.node.call_service("master/sdo_read", COReadID, rreq, rres)
        self.node.call_service("master/sdo_read", COReadID, rreq, rres)
        req.data = 0
        self.node.call_service("master/sdo_write", COWriteID, req, res)
        self.node.call_service("master/sdo_write", COWriteID, req, res)


class TestPDO(unittest.TestCase):
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

    def test_pdo(self):
        msg = COData()
        msg.index = 0x4000
        msg.subindex = 0
        msg.data = 200

        pub_msg = COData()
        pub_msg.index = 0x4001
        pub_msg.subindex = 0
        pub_msg.data = 200
        thread = threading.Thread(
            target=self.node.subscribe_and_wait_for_message,
            args=["proxy_device_1/rpdo", COData, pub_msg],
        )
        thread.start()
        self.node.publish_message("proxy_device_1/tpdo", COData, msg)
        time.sleep(0.1)
        msg.data = 0
        self.node.publish_message("proxy_device_1/tpdo", COData, msg)
        thread.join()
