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
from canopen_utils.launch_test_node import LaunchTestNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import unittest
from canopen_interfaces.srv import CORead, COWrite
from canopen_interfaces.msg import COData


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/proxy_lifecycle_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestLifecycle(unittest.TestCase):
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

    def test_configure_unconfigure(self):
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE

        res = ChangeState.Response()
        res.success = True

        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)

        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)

    def test_full_cycle(self):
        req = ChangeState.Request()

        res = ChangeState.Response()
        res.success = True

        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CLEANUP SUCCESSFUL*************************")

        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CLEANUP SUCCESSFUL*************************")


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

    def test_full_cycle_sdo(self):
        req = ChangeState.Request()

        res = ChangeState.Response()
        res.success = True

        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        data = 100

        readreq = CORead.Request()
        readreq.index = 0x4000
        readreq.subindex = 0x00

        readres = CORead.Response()
        readres.success = True
        readres.data = data

        writereq = COWrite.Request()
        writereq.index = 0x4000
        writereq.subindex = 0x00
        writereq.data = data

        writeres = COWrite.Response()
        writeres.success = True

        self.node.call_service("proxy_device_1/sdo_write", COWrite, writereq, writeres)
        self.node.call_service("proxy_device_2/sdo_write", COWrite, writereq, writeres)
        self.node.call_service("proxy_device_1/sdo_read", CORead, readreq, readres)
        self.node.call_service("proxy_device_2/sdo_read", CORead, readreq, readres)
        data = 0
        self.node.call_service("proxy_device_1/sdo_write", COWrite, writereq, writeres)
        self.node.call_service("proxy_device_2/sdo_write", COWrite, writereq, writeres)

        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CLEANUP SUCCESSFUL*************************")

        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        self.node.call_service("proxy_device_1/sdo_read", CORead, readreq, readres)
        self.node.call_service("proxy_device_2/sdo_read", CORead, readreq, readres)

        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CLEANUP SUCCESSFUL*************************")


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

    def test_full_cycle_sdo(self):
        req = ChangeState.Request()

        res = ChangeState.Response()
        res.success = True

        req.transition.id = Transition.TRANSITION_CONFIGURE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_ACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************ACTIVATE SUCCESSFUL*************************")

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

        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        req.transition.id = Transition.TRANSITION_CLEANUP
        self.node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)
        print("*************************CLEANUP SUCCESSFUL*************************")
