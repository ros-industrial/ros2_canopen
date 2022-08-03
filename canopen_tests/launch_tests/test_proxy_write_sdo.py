import os
import pytest
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import State
import unittest


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/proxy_write_sdo.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=10.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class MakeTestNode(Node):
    manager_name = "device_manager_node"

    def __init__(self, name="test_node"):
        super().__init__(name)

    def checkLifecycleServices(self) -> bool:
        nodes = self.get_node_names()
        for node in nodes:
            print(node)
        self.get_state_client = self.create_client(
            GetState, self.manager_name + "/get_state"
        )
        self.change_state_client = self.create_client(
            ChangeState, self.manager_name + "/change_state"
        )
        self.get_logger().info(
            "Checking for {}".format(self.manager_name + "/get_state")
        )
        if not self.get_state_client.wait_for_service(timeout_sec=3.0):
            return False
        if not self.change_state_client.wait_for_service(timeout_sec=3.0):
            return False
        return True

    def checkLifecycleConfigure(self) -> bool:
        req = GetState.Request()
        self.future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=3.0)
        if self.future.done():
            try:
                result = self.future.result()
            except Exception as e:
                return False
            else:
                if result.current_state.id != State.PRIMARY_STATE_UNCONFIGURED:
                    return False
                return True


class TestProxySdoWrite(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MakeTestNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_device_manager_lifecycle(self):
        assert self.node.checkLifecycleServices(), "Could not find lifecycle services"
        assert self.node.checkLifecycleConfigure(), "Could not configure device manager"
