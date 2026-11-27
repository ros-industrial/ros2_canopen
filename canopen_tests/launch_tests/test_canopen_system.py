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

"""CanopenSystem against fake basic slaves, driven through the CanopenProxyController."""

import os

import launch
import launch_testing
import pytest
from ament_index_python import get_package_share_directory
from canopen_interfaces.msg import COData
from canopen_utils.launch_test_node import LaunchTestCase
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from std_srvs.srv import Trigger

CONTROLLER = "/node_1_controller"


@pytest.mark.rostest
def generate_test_description():
    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("canopen_tests"), "launch", "canopen_system.launch.py"
            )
        )
    )
    ready_to_test = launch.actions.TimerAction(
        period=5.0, actions=[launch_testing.actions.ReadyToTest()]
    )
    return LaunchDescription([launch_desc, ready_to_test]), {}


# test methods run in name order and build on each other
class TestCanopenSystem(LaunchTestCase):

    def trigger(self, service):
        """Call an NMT service; success means the one-shot command reached the driver and its
        feedback came back through the command interfaces.

        The published NMT state is not checked: the controller publishes on change only, and no
        change is published after reset/start, with the pre-migration code as well.
        """
        response = self.node.call(f"{CONTROLLER}/{service}", Trigger, Trigger.Request(), 10.0)
        self.assertTrue(response.success, f"{service} failed: {response.message}")

    def test_1_controllers_active(self):
        # joint_state_broadcaster is not checked: CanopenSystem has no position/velocity/effort
        self.node.wait_for_active_controllers(["node_1_controller"])

    def test_2_nmt_reset(self):
        self.trigger("nmt_reset_node")

    def test_3_nmt_start(self):
        self.trigger("nmt_start_node")

    def test_4_repeated_commands(self):
        # each call must see fresh feedback, i.e. the hardware's writeback is not lost
        for _ in range(3):
            self.trigger("nmt_reset_node")
            self.trigger("nmt_start_node")

    def test_5_tpdo_reaches_device(self):
        # controller tpdo topic -> tpdo/* command interfaces -> master TPDO -> the fake slave
        # copies 0x4000 into its TPDO 0x4001, seen on the driver's own rpdo topic. The slave also
        # refreshes 0x4001 with 0x1122 at 10 Hz, so keep sending until the echo shows up.
        data = 0x2A2A
        publisher = self.node.create_publisher(COData, f"{CONTROLLER}/tpdo", 10)
        try:
            msg = self.node.wait_for_message(
                "/proxy_device_1/rpdo",
                COData,
                lambda m: m.index == 0x4001 and m.data == data,
                timeout=15.0,
                on_tick=lambda: publisher.publish(COData(index=0x4000, subindex=0, data=data)),
            )
        finally:
            self.node.destroy_publisher(publisher)
        self.assertIsNotNone(msg, "TPDO data sent through the controller did not reach the device")

    def test_6_rpdo_reaches_controller(self):
        # The slave's TPDO maps 0x4001 and 0x4004; CanopenSystem keeps the last object received
        # per node, so the controller reports 0x4004. Its initial message has index 0, so 0x4004
        # can only come from the rpdo/* state interfaces.
        msg = self.node.wait_for_message(
            f"{CONTROLLER}/rpdo", COData, lambda m: m.index == 0x4004, timeout=10.0
        )
        self.assertIsNotNone(msg, "RPDO from the device did not reach the controller")
