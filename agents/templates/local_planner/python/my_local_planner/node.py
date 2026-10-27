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
"""Local planner node for action_bringup (LOCAL_PLANNER=my_local_planner).

The interface is in agents/workflows/add-local-planner.md. You normally only edit planner.py.
Plans at plan_hz. Publishes /local_planner/path while plan() returns a path, and holds the latched
/local_planner/active flag true only then. With stale inputs or no path the flag goes false and
the controller falls back to the global raceline.
"""

import dataclasses
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from my_local_planner.planner import Params, State, plan

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
RELIABLE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class MyLocalPlannerNode(Node):
    def __init__(self):
        super().__init__("my_local_planner_node")
        # Topics follow the contract (contract.yaml); the values live in config/{car,sim}
        self.declare_parameter("odom_topic", "/hal/localization/odom")
        self.declare_parameter("speed_topic", "/hal/odom")
        self.declare_parameter("scan_topic", "/hal/scan")
        self.declare_parameter("global_path_topic", "/global_planner/path")
        self.declare_parameter("path_topic", "/local_planner/path")
        self.declare_parameter("active_topic", "/local_planner/active")
        self.declare_parameter("plan_hz", 20.0)
        self.declare_parameter("stale_s", 0.5)
        for f in dataclasses.fields(Params):
            self.declare_parameter(f.name, f.default)

        p = self.get_parameter
        self.params = Params(
            **{f.name: p(f.name).value for f in dataclasses.fields(Params)}
        )
        self.stale_s = p("stale_s").value

        self.state = State()
        self.scan = None
        self.global_path = []
        self.last_odom = None
        self.last_scan = None
        self.active = None

        self.path_pub = self.create_publisher(Path, p("path_topic").value, RELIABLE)
        self.active_pub = self.create_publisher(Bool, p("active_topic").value, LATCHED)
        self.create_subscription(Odometry, p("odom_topic").value, self.on_odom, 10)
        self.create_subscription(Odometry, p("speed_topic").value, self.on_speed, 10)
        # sensor-data QoS (best effort) matches both reliable and best-effort scan publishers
        self.create_subscription(
            LaserScan, p("scan_topic").value, self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Path, p("global_path_topic").value, self.on_global_path, LATCHED
        )

        self.set_active(False)
        self.create_timer(1.0 / p("plan_hz").value, self.on_timer)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        self.state.x = pose.position.x
        self.state.y = pose.position.y
        self.state.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.last_odom = self.now_s()

    def on_speed(self, msg):
        self.state.speed = msg.twist.twist.linear.x

    def on_scan(self, msg):
        self.scan = (list(msg.ranges), msg.angle_min, msg.angle_increment)
        self.last_scan = self.now_s()

    def on_global_path(self, msg):
        self.global_path = [
            (p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses
        ]

    def on_timer(self):
        t = self.now_s()
        fresh = all(
            s is not None and t - s < self.stale_s
            for s in (self.last_odom, self.last_scan)
        )
        if not fresh or not self.global_path:
            self.get_logger().warn(
                "waiting for localization, scan and the global path",
                throttle_duration_sec=2.0,
            )
            self.set_active(False)
            return

        path = plan(self.state, self.scan, self.global_path, self.params)
        if not path:
            self.set_active(False)
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        for x, y, v in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(v)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.set_active(True)

    def set_active(self, active):
        # Latched, so only send changes
        if self.active == active:
            return
        self.active = active
        self.active_pub.publish(Bool(data=active))


def main():
    rclpy.init()
    node = MyLocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
