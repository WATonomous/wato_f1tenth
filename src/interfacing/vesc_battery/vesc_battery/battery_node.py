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
"""VESC telemetry (/sensors/core) -> sensor_msgs/BatteryState on /hal/battery.

Logs a warning when a cell drops below warn_cell_v and an error below critical_cell_v. The Jetson
runs off the same pack, so a critical battery means the whole car is about to brown out.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from vesc_msgs.msg import VescStateStamped

from vesc_battery import estimate

NAN = float("nan")


class BatteryNode(Node):
    def __init__(self):
        super().__init__("vesc_battery")
        self.declare_parameter("input_topic", "/sensors/core")
        self.declare_parameter("output_topic", "/hal/battery")
        self.declare_parameter("cells", 3)
        self.declare_parameter("capacity_ah", 5.0)
        self.declare_parameter("internal_resistance_ohm", 0.03)
        self.declare_parameter("smoothing_s", 3.0)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("warn_cell_v", 3.7)
        self.declare_parameter("critical_cell_v", 3.6)
        # Below this per cell the reading isn't a battery (VESC on USB power only, or unplugged)
        self.declare_parameter("present_min_cell_v", 2.5)
        self.declare_parameter("stale_s", 1.0)

        p = self.get_parameter
        self.cells = p("cells").value
        self.capacity = p("capacity_ah").value
        self.r_int = p("internal_resistance_ohm").value
        self.warn = p("warn_cell_v").value
        self.critical = p("critical_cell_v").value
        self.present_min = p("present_min_cell_v").value
        self.stale_s = p("stale_s").value

        self.voltage = estimate.Smoother(p("smoothing_s").value)
        self.resting = estimate.Smoother(p("smoothing_s").value)
        self.current = NAN
        self.raw_voltage = NAN
        self.last_rx = None
        self.last_level = None  # so the first reading is logged

        self.pub = self.create_publisher(BatteryState, p("output_topic").value, 10)
        self.sub = self.create_subscription(
            VescStateStamped, p("input_topic").value, self.on_state, 10
        )
        self.timer = self.create_timer(1.0 / p("publish_hz").value, self.publish)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        t = self.now_s()
        s = msg.state
        self.current = s.current_input
        self.raw_voltage = s.voltage_input
        self.last_rx = t
        if s.voltage_input / self.cells < self.present_min:
            # Not a battery (VESC on USB power only): drop the history so that reconnecting, or
            # this drop itself, never slides the smoothed value through the low/critical range
            self.voltage.reset()
            self.resting.reset()
            return
        self.voltage.update(s.voltage_input, t)
        self.resting.update(
            estimate.resting_cell_voltage(
                s.voltage_input, s.current_input, self.cells, self.r_int
            ),
            t,
        )

    def publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.design_capacity = self.capacity
        msg.capacity = self.capacity
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.location = "drive pack (current: VESC input only, Jetson load not included)"

        fresh = self.last_rx is not None and self.now_s() - self.last_rx < self.stale_s
        cell_v = self.resting.value if fresh else None
        msg.present = bool(fresh and cell_v is not None)
        if not msg.present:
            msg.voltage = self.raw_voltage if fresh else NAN
            msg.current = msg.charge = msg.percentage = NAN
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            self.pub.publish(msg)
            return

        fraction = estimate.cell_fraction(cell_v)
        msg.voltage = self.voltage.value
        # BatteryState convention: negative while discharging
        msg.current = -self.current if not math.isnan(self.current) else NAN
        msg.percentage = fraction
        msg.charge = fraction * self.capacity
        msg.cell_voltage = [self.voltage.value / self.cells] * self.cells
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        lvl = estimate.level(cell_v, self.warn, self.critical)
        msg.power_supply_health = {
            estimate.OK: BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            estimate.LOW: BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            estimate.CRITICAL: BatteryState.POWER_SUPPLY_HEALTH_DEAD,  # no "low" health in BatteryState
            estimate.OVERVOLTAGE: BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE,
        }[lvl]
        self.pub.publish(msg)
        self.report(lvl, msg.voltage, fraction)

    def report(self, lvl, pack_v, fraction):
        text = f"battery {pack_v:.2f} V ({fraction * 100:.0f}%)"
        if lvl == estimate.CRITICAL:
            self.get_logger().error(
                f"{text}: CRITICAL, the Jetson shares this pack; stop and charge",
                throttle_duration_sec=5.0,
            )
        elif lvl == estimate.LOW:
            self.get_logger().warn(
                f"{text}: low, charge soon", throttle_duration_sec=30.0
            )
        elif lvl == estimate.OVERVOLTAGE:
            self.get_logger().warn(
                f"{text}: above 4.25 V per cell, check the cell count",
                throttle_duration_sec=30.0,
            )
        elif lvl != self.last_level:
            self.get_logger().info(text)
        self.last_level = lvl


def main():
    rclpy.init()
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
