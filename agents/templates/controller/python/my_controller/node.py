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
"""Controller node for action_bringup (CONTROLLER=my_controller).

The interface is in agents/workflows/add-controller.md. You normally only edit control.py.
Runs one control step per localization update (rate-limited to control_hz). Publishes a stop when
the dead man switch is released, when there is no path, and from a watchdog when localization
goes stale.
"""

import dataclasses
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from my_controller.control import Params, State, compute

# Latched topics (global path, local "active" flag) were published before this node started
LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
# Volatile + reliable matches the gamepad's latched publisher and plain ones such as Foxglove
RELIABLE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
DEBUG = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def as_xyv(path):
    return [
        (p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path.poses
    ]


class MyControllerNode(Node):
    def __init__(self):
        super().__init__("my_controller_node")
        # Topics follow the contract (contract.yaml); the values live in config/{car,sim}
        self.declare_parameter("odom_topic", "/hal/localization/odom")
        self.declare_parameter("speed_topic", "/hal/odom")
        self.declare_parameter("global_path_topic", "/global_planner/path")
        self.declare_parameter("local_path_topic", "/local_planner/path")
        self.declare_parameter("local_active_topic", "/local_planner/active")
        self.declare_parameter("dead_man_topic", "/hal/dead_man_switch")
        self.declare_parameter("drive_topic", "/hal/drive/autonomy")
        self.declare_parameter("debug_topic", "/debug/my_controller/target")
        # Set by the launch file: true when LOCAL_PLANNER is not none
        self.declare_parameter("use_local_planner", False)
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("stale_s", 0.5)
        self.declare_parameter("enable_debug_vis", False)
        for f in dataclasses.fields(Params):
            self.declare_parameter(f.name, f.default)

        p = self.get_parameter
        self.params = Params(
            **{f.name: p(f.name).value for f in dataclasses.fields(Params)}
        )
        self.use_local = p("use_local_planner").value
        self.period = 1.0 / p("control_hz").value
        self.stale_s = p("stale_s").value
        self.debug_vis = p("enable_debug_vis").value

        self.state = State()
        self.global_path = []
        self.global_frame = "map"
        self.local_path = []
        self.local_frame = "base_link"
        self.local_path_rx = None
        self.local_active = False
        self.dead_man = False
        self.last_odom = None
        self.last_step = None

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, p("drive_topic").value, 10
        )
        self.debug_pub = self.create_publisher(Marker, p("debug_topic").value, DEBUG)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(Odometry, p("speed_topic").value, self.on_speed, 10)
        self.create_subscription(
            Path, p("global_path_topic").value, self.on_global_path, LATCHED
        )
        self.create_subscription(
            Bool, p("dead_man_topic").value, self.on_dead_man, RELIABLE
        )
        if self.use_local:
            self.create_subscription(
                Path, p("local_path_topic").value, self.on_local_path, RELIABLE
            )
            self.create_subscription(
                Bool, p("local_active_topic").value, self.on_local_active, LATCHED
            )
        self.create_timer(0.1, self.on_watchdog)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_speed(self, msg):
        self.state.speed = msg.twist.twist.linear.x

    def on_global_path(self, msg):
        self.global_path = as_xyv(msg)
        self.global_frame = msg.header.frame_id

    def on_local_path(self, msg):
        self.local_path = as_xyv(msg)
        self.local_frame = msg.header.frame_id
        self.local_path_rx = self.now_s()

    def on_local_active(self, msg):
        self.local_active = msg.data

    def on_dead_man(self, msg):
        self.dead_man = msg.data

    def on_odom(self, msg):
        t = self.now_s()
        self.last_odom = t
        if self.last_step is not None and t - self.last_step < self.period:
            return
        self.last_step = t

        pose = msg.pose.pose
        q = pose.orientation
        self.state.x = pose.position.x
        self.state.y = pose.position.y
        self.state.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        if not self.dead_man:
            self.publish(0.0, 0.0)
            return

        # Follow the local planner's path while it is active and fresh, else the global raceline
        local = (
            self.use_local
            and self.local_active
            and bool(self.local_path)
            and self.local_path_rx is not None
            and t - self.local_path_rx < self.stale_s
        )
        path, frame = (
            (self.local_path, self.local_frame)
            if local
            else (self.global_path, self.global_frame)
        )
        if not path:
            self.get_logger().warn(
                "no path yet, holding the car stopped", throttle_duration_sec=2.0
            )
            self.publish(0.0, 0.0)
            return

        cmd = compute(self.state, path, local, self.params)
        limit = self.params.max_steering_angle
        self.publish(
            min(max(cmd.steering_angle, -limit), limit),
            min(max(cmd.speed, 0.0), self.params.max_speed),
        )
        if self.debug_vis and cmd.debug_point is not None:
            self.publish_debug(frame, *cmd.debug_point)

    def on_watchdog(self):
        if self.last_odom is None or self.now_s() - self.last_odom > self.stale_s:
            self.get_logger().warn(
                "no localization, holding the car stopped", throttle_duration_sec=2.0
            )
            self.publish(0.0, 0.0)

    def publish(self, steering_angle, speed):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.drive.steering_angle = float(steering_angle)
        out.drive.speed = float(speed)
        self.drive_pub.publish(out)

    def publish_debug(self, frame_id, x, y):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.ns = "my_controller"
        m.type = Marker.SPHERE
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = m.color.a = 1.0
        self.debug_pub.publish(m)


def main():
    rclpy.init()
    node = MyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
