# Copyright (c) 2025-present WATonomous. All rights reserved.
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
"""drive_status: who has control of the car, on /control/status at 10 Hz.

Combines the ackermann_mux input in control (/control/active_input), the gamepad mode
(/control/mode), the dead man, the command reaching the motor and the measured speed, plus the
CONTROLLER / LOCAL_PLANNER / TRACK names, into one message for the status panel.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_msgs.msg import ControlStatus
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from drive_status.state import control_state

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
# Volatile + reliable matches the gamepad's latched dead man and plain publishers alike
RELIABLE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class DriveStatus(Node):
    def __init__(self):
        super().__init__("drive_status")
        self.declare_parameter("status_topic", "/control/status")
        self.declare_parameter("active_input_topic", "/control/active_input")
        self.declare_parameter("mode_topic", "/control/mode")
        self.declare_parameter("dead_man_topic", "/hal/dead_man_switch")
        self.declare_parameter("command_topic", "/hal/ackermann_cmd")
        self.declare_parameter("odom_topic", "/hal/odom")
        self.declare_parameter("rate_hz", 10.0)
        # Command older than this: shown as 0 (the mux stopped sending)
        self.declare_parameter("command_stale_s", 0.5)
        # Set by interfacing.launch.yaml from CONTROLLER, LOCAL_PLANNER, TRACK
        self.declare_parameter("controller", "")
        self.declare_parameter("local_planner", "")
        self.declare_parameter("track", "")

        p = self.get_parameter
        self.stale_s = p("command_stale_s").value
        self.msg = ControlStatus(
            mode="UNKNOWN",
            active_input="none",
            controller=p("controller").value,
            local_planner=p("local_planner").value,
            track=p("track").value,
            measured_speed=math.nan,
        )
        self.command_rx = None

        self.pub = self.create_publisher(ControlStatus, p("status_topic").value, 10)
        self.create_subscription(
            String, p("active_input_topic").value, self.on_active, LATCHED
        )
        self.create_subscription(String, p("mode_topic").value, self.on_mode, LATCHED)
        self.create_subscription(
            Bool, p("dead_man_topic").value, self.on_dead_man, RELIABLE
        )
        self.create_subscription(
            AckermannDriveStamped, p("command_topic").value, self.on_command, 10
        )
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_timer(1.0 / p("rate_hz").value, self.publish)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_active(self, msg):
        self.msg.active_input = msg.data

    def on_mode(self, msg):
        self.msg.mode = msg.data

    def on_dead_man(self, msg):
        self.msg.dead_man = msg.data

    def on_command(self, msg):
        self.msg.commanded_speed = msg.drive.speed
        self.msg.commanded_steering = msg.drive.steering_angle
        self.command_rx = self.now_s()

    def on_odom(self, msg):
        self.msg.measured_speed = msg.twist.twist.linear.x

    def publish(self):
        if self.command_rx is None or self.now_s() - self.command_rx > self.stale_s:
            self.msg.commanded_speed = 0.0
            self.msg.commanded_steering = 0.0
        self.msg.state = control_state(self.msg.active_input, self.msg.dead_man)
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = DriveStatus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
