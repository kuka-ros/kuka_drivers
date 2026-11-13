# Copyright 2024 KUKA Hungaria Kft.
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

import os
import time
import unittest
import xml.etree.ElementTree as ET

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# Launch all of the robot visualisation launch files one by one
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("kuka_rsi_driver"),
                        "/launch/",
                        "startup.launch.py",
                    ]
                ),
                launch_arguments={
                    "driver_version": "rsi_only",
                    "robot_xacro_file": os.path.join(
                        get_package_share_directory("kuka_rsi_driver"),
                        "test",
                        "fixture with spaces",
                        "cell_with_external_base.urdf.xacro",
                    ),
                }.items(),
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestDriverStartup(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        # Check for successful initialization
        proc_output.assertWaitFor("Robot initialized", timeout=5)
        proc_output.assertWaitFor("Successful initialization of hardware 'kr6_r700_2'", timeout=5)
        # Check whether disabling automatic activation was successful
        proc_output.assertWaitFor(
            "Setting component 'kr6_r700_2' to 'unconfigured' state.", timeout=5
        )

    def test_robot_description_contains_external_base(self):
        robot_descriptions = []
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        rclpy.init()
        node = rclpy.create_node("test_robot_description")
        node.create_subscription(
            String,
            "/robot_description",
            lambda message: robot_descriptions.append(message.data),
            qos_profile,
        )
        try:
            deadline = time.monotonic() + 10
            while not robot_descriptions and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

            self.assertTrue(robot_descriptions, "robot_description was not published")
            robot = ET.fromstring(robot_descriptions[-1])
            self.assertIsNotNone(robot.find("./link[@name='work_base']"))
            self.assertIsNotNone(
                robot.find("./joint[@name='world-work_base']/child[@link='work_base']")
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
