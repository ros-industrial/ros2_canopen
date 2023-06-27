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

import time
import re

import launch_testing
import launch_testing.asserts
import launch_testing.actions
import launch_testing.util
import launch_testing.tools
from launch_testing.asserts import assertSequentialStdout

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.client import Client
from rclpy.qos import QoSProfile
from threading import Condition


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

    def subscribe_and_wait_for_message(self, topic: str, msg_type, msg, timeout: float = 1.0):
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
