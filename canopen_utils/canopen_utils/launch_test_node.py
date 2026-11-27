# Copyright 2023 ROS-Industrial
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time
import unittest
import re

import launch_testing
import launch_testing.asserts
import launch_testing.actions
import launch_testing.util
import launch_testing.tools
from launch_testing.asserts import assertSequentialStdout

import rclpy
import rclpy.executors
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.qos import QoSProfile
from threading import Condition
from sensor_msgs.msg import JointState


class Ros2ActiveIoHandler:
    def __init__(self, proc_output: launch_testing.ActiveIoHandler):
        self.proc_output = proc_output

    def checkInRos2Stream(self, process, expected_output: str):
        resolved_procs = launch_testing.util.resolveProcesses(
            info_obj=self.proc_output, process=process, cmd_args=None, strict_proc_matching=True
        )

        pat = re.compile(r".*\]: " + expected_output)

        for proc in resolved_procs:
            for output in self.proc_output[proc]:
                text = output.text.decode()
                print("recceived: " + text)
                res = pat.search(text)
                if res is not None:
                    return True
        return False

    def waitFor(self, process, expected_output: str, timeout: float = 1.0):
        success = False

        with self.proc_output._sync_lock:
            # TODO(pete.baughman): Searching through all of the IO can be time consuming/wasteful.
            # We can optimize this by making a note of where we left off searching and only
            # searching new messages when we return from the wait.
            success = self.proc_output._sync_lock.wait_for(
                lambda: self.checkInRos2Stream(process=process, expected_output=expected_output),
                timeout=timeout,
            )

        return success

    def assertWaitFor(self, process, expected_output: str, timeout: float = 1.0):
        success = self.waitFor(process=process, expected_output=expected_output, timeout=timeout)
        assert success, f"Waiting for output {expected_output} timed out"


class LaunchTestNode(Node):
    def __init__(self):
        super().__init__("launch_test_node")
        self.get_logger().info("launch_test_node initialized")

    def publish_delayed(self, publisher: Publisher, msg, delay: float):
        publisher.publish(msg)
        time.sleep(delay)

    def publish_and_check_output(
        self,
        publisher: Publisher,
        msg,
        expected_output: str,
        proc_output: launch_testing.ActiveIoHandler,
        process=None,
    ):

        self.publish_delayed(publisher, msg, 0.1)
        Ros2ActiveIoHandler(proc_output).assertWaitFor(
            process=process, expected_output=expected_output
        )

    def subscribe_and_wait_for_message(self, topic: str, msg_type, msg, timeout: float = 2.5):
        condition = Condition()
        self.received = False

        def callback(rec_msg):
            with condition:
                if not self.received:
                    self.received = msg == rec_msg
                if self.received:
                    condition.notify()

        subscription: Subscription = self.create_subscription(msg_type, topic, callback, 1)
        with condition:
            condition.wait(timeout=timeout)
        self.destroy_subscription(subscription)
        assert self.received, "Did not receive the expected message."

    def call_service(
        self, service_name: str, srv_type, service_request, expected_response, timeout: float = 2.0
    ):
        condition = Condition()

        def callback(arg):
            with condition:
                condition.notify()

        client: Client = self.create_client(
            srv_type, service_name, qos_profile=QoSProfile(depth=1)
        )
        if not client.wait_for_service(timeout_sec=timeout):
            assert False, "Service server not available."
        future = client.call_async(service_request)
        future.add_done_callback(callback=callback)
        with condition:
            if not condition.wait(timeout=timeout):
                assert False, "Service call timed out."

        res = future.result()
        assert res == expected_response, "Did not receive the expected response."
        self.destroy_client(client)

    def publish_message(self, topic_name: str, topic_type, msg):
        publisher = self.create_publisher(topic_type, topic_name, 10)
        publisher.publish(msg)
        self.destroy_publisher(publisher)

    def call(self, service_name: str, srv_type, request, timeout: float = 5.0):
        """Call a service and return its response; fails the test on timeout."""
        client: Client = self.create_client(srv_type, service_name)
        try:
            assert client.wait_for_service(timeout_sec=timeout), f"{service_name} not available"
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            assert future.done(), f"{service_name} call timed out"
            return future.result()
        finally:
            self.destroy_client(client)

    def wait_for_message(
        self, topic: str, msg_type, predicate, timeout: float = 5.0, on_tick=None
    ):
        """Return the first message on topic satisfying predicate, or None after timeout.

        on_tick is called every 100 ms while waiting, e.g. to keep publishing a command.
        """
        condition = Condition()
        matched = []

        def callback(msg):
            if not matched and predicate(msg):
                with condition:
                    matched.append(msg)
                    condition.notify()

        subscription: Subscription = self.create_subscription(msg_type, topic, callback, 10)
        try:
            deadline = time.monotonic() + timeout
            with condition:
                while not matched and time.monotonic() < deadline:
                    if on_tick:
                        on_tick()
                    condition.wait(timeout=0.1)
            return matched[0] if matched else None
        finally:
            self.destroy_subscription(subscription)

    def wait_for_active_controllers(self, names, timeout: float = 30.0):
        """Wait until all named controllers are active in /controller_manager."""
        deadline = time.monotonic() + timeout
        active = set()
        while time.monotonic() < deadline:
            response = self.call(
                "/controller_manager/list_controllers", ListControllers, ListControllers.Request()
            )
            active = {c.name for c in response.controller if c.state == "active"}
            if set(names) <= active:
                return
            time.sleep(0.5)
        assert False, f"controllers not active: {set(names) - active}"

    def joint_position(self, joint: str, timeout: float = 5.0, predicate=None, on_tick=None):
        """Wait for /joint_states with joint's position satisfying predicate; return it."""

        def matches(msg):
            if joint not in msg.name:
                return False
            position = msg.position[msg.name.index(joint)]
            return predicate is None or predicate(position)

        msg = self.wait_for_message("/joint_states", JointState, matches, timeout, on_tick)
        return None if msg is None else msg.position[msg.name.index(joint)]


class LaunchTestCase(unittest.TestCase):
    """Test case with one LaunchTestNode spun in a background thread for the whole class."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()
