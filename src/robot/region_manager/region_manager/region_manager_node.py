#!/usr/bin/env python3
"""
RegionManager — publishes the active track region for bridge handling.

Subscribes to PF pose. When the car enters a "switch to OVER" bubble, publishes
region=OVER. When the car enters a "switch to UNDER" bubble, publishes
region=UNDER. State machine with hysteresis: each bubble fires only the
appropriate direction so there's no flapping at the trigger boundary.

Topics:
    in:  pose_topic        (default /pf/pose/odom, nav_msgs/Odometry)
    out: /region/active    (std_msgs/String — "under" or "over")
         /region/event     (std_msgs/String — one-shot "enter_<region>" on transitions, for logging)

Params:
    pose_topic                 default /pf/pose/odom
    publish_rate_hz            default 30.0 (republishes current state for late subscribers)
    initial_region             default "under"
    # Bubble A: triggers UNDER → OVER
    enter_over_x               (m)
    enter_over_y               (m)
    enter_over_radius          (m)
    # Bubble B: triggers OVER → UNDER
    enter_under_x              (m)
    enter_under_y              (m)
    enter_under_radius         (m)
    # Optional: force one-shot ignore until first PF pose received
    require_pose_before_publish  default True
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, ColorRGBA
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


REGION_UNDER = "under"
REGION_OVER = "over"


class RegionManager(Node):
    def __init__(self):
        super().__init__("region_manager")

        # Params
        self.declare_parameter("pose_topic", "/pf/pose/odom")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("initial_region", REGION_UNDER)
        self.declare_parameter("enter_over_x", 0.0)
        self.declare_parameter("enter_over_y", 0.0)
        self.declare_parameter("enter_over_radius", 0.5)
        self.declare_parameter("enter_under_x", 0.0)
        self.declare_parameter("enter_under_y", 0.0)
        self.declare_parameter("enter_under_radius", 0.5)
        self.declare_parameter("require_pose_before_publish", True)
        self.declare_parameter("publish_bubble_viz", True)
        self.declare_parameter("bubble_viz_frame_id", "map")

        self.pose_topic = self.get_parameter("pose_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.region = str(self.get_parameter("initial_region").value).strip().lower()
        if self.region not in (REGION_UNDER, REGION_OVER):
            self.get_logger().warn(f"initial_region '{self.region}' invalid, defaulting to under")
            self.region = REGION_UNDER

        self.enter_over = self._read_bubble("enter_over")
        self.enter_under = self._read_bubble("enter_under")
        self.require_pose = bool(self.get_parameter("require_pose_before_publish").value)
        self.publish_bubble_viz = bool(self.get_parameter("publish_bubble_viz").value)
        self.bubble_viz_frame_id = str(self.get_parameter("bubble_viz_frame_id").value)

        # Latched + reliable QoS so late subscribers (MPPI / PF) catch the
        # current state on connect — important if the node started before them.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.region_pub = self.create_publisher(String, "/region/active", latched_qos)
        self.event_pub = self.create_publisher(String, "/region/event", 10)
        # Bubble viz: latched so rviz/foxglove see it as soon as they connect.
        self.bubble_viz_pub = self.create_publisher(MarkerArray, "/region/bubbles_viz", latched_qos)

        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self.on_pose, 10)

        self.have_pose = False
        self.last_pose_xy = (0.0, 0.0)
        # State machine: keep latest region; once we're "over", only the
        # enter_under bubble can flip us back, and vice-versa.

        # Periodic republish (latched QoS handles late joiners but this keeps
        # /region/active warm and visible in foxglove)
        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.create_timer(period, self.tick_publish)

        self.get_logger().info(
            f"RegionManager up | region='{self.region}' | "
            f"OVER bubble=({self.enter_over[0]:.2f},{self.enter_over[1]:.2f},r={self.enter_over[2]:.2f}) | "
            f"UNDER bubble=({self.enter_under[0]:.2f},{self.enter_under[1]:.2f},r={self.enter_under[2]:.2f})"
        )

        # Publish initial state immediately (if not pose-gated)
        if not self.require_pose:
            self._publish_region()

        # One-shot bubble viz on the latched topic
        if self.publish_bubble_viz:
            self._publish_bubble_markers()

    def _publish_bubble_markers(self):
        """Publish two CYLINDER markers (with optional TEXT labels) representing
        the OVER and UNDER trigger bubbles. Use a flat z so they render as discs
        when viewed from above in rviz / foxglove 3D panels."""
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # (label, color, bubble) — green = enters OVER, blue = enters UNDER
        configs = [
            ("enter_over", (0.10, 0.80, 0.30, 0.45), self.enter_over),
            ("enter_under", (0.20, 0.55, 0.95, 0.45), self.enter_under),
        ]
        mid = 0
        for label, rgba, bubble in configs:
            x, y, r = bubble
            # Cylinder body (flat disc)
            m = Marker()
            m.header.frame_id = self.bubble_viz_frame_id
            m.header.stamp = now
            m.ns = "region_bubbles"
            m.id = mid; mid += 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = float(2.0 * r)
            m.scale.y = float(2.0 * r)
            m.scale.z = 0.04
            m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]),
                                b=float(rgba[2]), a=float(rgba[3]))
            arr.markers.append(m)
            # Text label above
            txt = Marker()
            txt.header.frame_id = self.bubble_viz_frame_id
            txt.header.stamp = now
            txt.ns = "region_bubbles"
            txt.id = mid; mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(x)
            txt.pose.position.y = float(y)
            txt.pose.position.z = 0.50
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.25
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f"{label} (r={r:.2f})"
            arr.markers.append(txt)
        self.bubble_viz_pub.publish(arr)

    def _read_bubble(self, prefix):
        return (
            float(self.get_parameter(f"{prefix}_x").value),
            float(self.get_parameter(f"{prefix}_y").value),
            float(self.get_parameter(f"{prefix}_radius").value),
        )

    def on_pose(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_pose_xy = (x, y)
        self.have_pose = True

        # Hysteresis: each bubble only fires the corresponding direction.
        if self.region == REGION_UNDER:
            if self._inside_bubble(x, y, self.enter_over):
                self._transition(REGION_OVER)
        else:
            if self._inside_bubble(x, y, self.enter_under):
                self._transition(REGION_UNDER)

    @staticmethod
    def _inside_bubble(x, y, bubble):
        bx, by, br = bubble
        dx = x - bx
        dy = y - by
        return (dx * dx + dy * dy) <= (br * br)

    def _transition(self, new_region):
        if new_region == self.region:
            return
        self.get_logger().info(
            f"REGION SWITCH: {self.region} → {new_region} at "
            f"({self.last_pose_xy[0]:.2f}, {self.last_pose_xy[1]:.2f})"
        )
        self.region = new_region
        self._publish_region()
        # Also emit one-shot event for logging / opp silence triggers downstream
        evt = String()
        evt.data = f"enter_{new_region}"
        self.event_pub.publish(evt)

    def _publish_region(self):
        msg = String()
        msg.data = self.region
        self.region_pub.publish(msg)

    def tick_publish(self):
        if self.require_pose and not self.have_pose:
            return
        self._publish_region()


def main(args=None):
    rclpy.init(args=args)
    node = RegionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
