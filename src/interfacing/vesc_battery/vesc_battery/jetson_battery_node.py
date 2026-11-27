# Copyright (c) 2026-present WATonomous. All rights reserved.
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
"""INA226 I2C 4S pack monitor -> sensor_msgs/BatteryState on /hal/jetson_battery.

Reads the Jetson's dedicated 4S LiPo pack voltage via a TI INA226 connected to the pack's
JST-XH 5-pin balance plug (Pin 1 = B-, Pin 5 = B4+) on the Jetson 40-pin I2C header
(/dev/i2c-7). Fails soft (publishes present=False) when the sensor or battery is unplugged.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from vesc_battery import estimate
from vesc_battery.ina226 import INA226

NAN = float("nan")


class JetsonBatteryMonitor:
    """ROS-free state estimator for the dedicated 4S Jetson battery pack."""

    def __init__(
        self,
        cells: int = 4,
        capacity_ah: float = 5.0,
        internal_resistance_ohm: float = 0.04,
        estimated_load_a: float = 0.0,
        smoothing_s: float = 3.0,
        warn_cell_v: float = 3.65,
        critical_cell_v: float = 3.50,
        present_min_cell_v: float = 2.5,
    ):
        self.cells = cells
        self.capacity_ah = capacity_ah
        self.r_int = internal_resistance_ohm
        self.estimated_load_a = estimated_load_a
        self.warn_cell_v = warn_cell_v
        self.critical_cell_v = critical_cell_v
        self.present_min_cell_v = present_min_cell_v
        self.voltage = estimate.Smoother(smoothing_s)
        self.resting = estimate.Smoother(smoothing_s)

    def reset(self) -> None:
        self.voltage.reset()
        self.resting.reset()

    def update(self, pack_v: float, now_s: float) -> dict:
        """Process one raw pack voltage reading and return a structured BatteryState snapshot."""
        if (
            pack_v is None
            or math.isnan(pack_v)
            or pack_v / self.cells < self.present_min_cell_v
        ):
            self.reset()
            return {
                "present": False,
                "voltage": pack_v
                if (pack_v is not None and not math.isnan(pack_v))
                else NAN,
                "cell_v": NAN,
                "percentage": NAN,
                "charge": NAN,
                "level": None,
            }

        smoothed_v = self.voltage.update(pack_v, now_s)
        rest_cell_v = self.resting.update(
            estimate.resting_cell_voltage(
                pack_v, self.estimated_load_a, self.cells, self.r_int
            ),
            now_s,
        )
        fraction = estimate.cell_fraction(rest_cell_v)
        lvl = estimate.level(rest_cell_v, self.warn_cell_v, self.critical_cell_v)
        return {
            "present": True,
            "voltage": smoothed_v,
            "cell_v": smoothed_v / self.cells,
            "percentage": fraction,
            "charge": fraction * self.capacity_ah,
            "level": lvl,
        }


class JetsonBatteryNode(Node):
    def __init__(self, sensor=None):
        super().__init__("jetson_battery")
        self.declare_parameter("output_topic", "/hal/jetson_battery")
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_address", 0x40)
        self.declare_parameter("voltage_scale", 1.0012)
        self.declare_parameter("verify_chip_id", True)
        self.declare_parameter("cells", 4)
        self.declare_parameter("capacity_ah", 5.0)
        self.declare_parameter("internal_resistance_ohm", 0.04)
        self.declare_parameter("estimated_load_a", 0.0)
        self.declare_parameter("smoothing_s", 3.0)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("warn_cell_v", 3.65)
        self.declare_parameter("critical_cell_v", 3.50)
        self.declare_parameter("present_min_cell_v", 2.5)

        p = self.get_parameter
        self.cells = int(p("cells").value)
        self.capacity = float(p("capacity_ah").value)
        self.estimated_load_a = float(p("estimated_load_a").value)
        self.last_level = None

        self.monitor = JetsonBatteryMonitor(
            cells=self.cells,
            capacity_ah=self.capacity,
            internal_resistance_ohm=float(p("internal_resistance_ohm").value),
            estimated_load_a=self.estimated_load_a,
            smoothing_s=float(p("smoothing_s").value),
            warn_cell_v=float(p("warn_cell_v").value),
            critical_cell_v=float(p("critical_cell_v").value),
            present_min_cell_v=float(p("present_min_cell_v").value),
        )
        self.sensor = sensor or INA226(
            bus=int(p("i2c_bus").value),
            address=int(p("i2c_address").value),
            voltage_scale=float(p("voltage_scale").value),
            verify_chip_id=bool(p("verify_chip_id").value),
        )

        self.pub = self.create_publisher(BatteryState, p("output_topic").value, 10)
        self.timer = self.create_timer(
            1.0 / float(p("publish_hz").value), self.poll_and_publish
        )
        self.get_logger().info(
            f"jetson_battery: INA226 on /dev/i2c-{int(p('i2c_bus').value)} "
            f"0x{int(p('i2c_address').value):02x} ({self.cells}S, scale={float(p('voltage_scale').value):.4f})"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def poll_and_publish(self) -> None:
        raw_v = NAN
        try:
            raw_v = self.sensor.read_bus_voltage()
        except OSError as exc:
            self.get_logger().warn(
                f"jetson_battery: INA226 read failed ({exc}); publishing present=False",
                throttle_duration_sec=30.0,
            )

        snap = self.monitor.update(raw_v, self.now_s())
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.design_capacity = self.capacity
        msg.capacity = self.capacity
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.location = "jetson 4S pack (INA226 balance tap)"
        msg.present = snap["present"]

        if not snap["present"]:
            msg.voltage = float(snap["voltage"])
            msg.current = msg.charge = msg.percentage = NAN
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            self.pub.publish(msg)
            return

        msg.voltage = float(snap["voltage"])
        msg.current = -self.estimated_load_a if self.estimated_load_a > 0.0 else NAN
        msg.percentage = float(snap["percentage"])
        msg.charge = float(snap["charge"])
        msg.cell_voltage = [float(snap["cell_v"])] * self.cells
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = {
            estimate.OK: BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            estimate.LOW: BatteryState.POWER_SUPPLY_HEALTH_GOOD,
            estimate.CRITICAL: BatteryState.POWER_SUPPLY_HEALTH_DEAD,
            estimate.OVERVOLTAGE: BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE,
        }[snap["level"]]
        self.pub.publish(msg)
        self.report(snap["level"], msg.voltage, msg.percentage)

    def report(self, lvl: str, pack_v: float, fraction: float) -> None:
        text = f"jetson battery {pack_v:.2f} V ({fraction * 100:.0f}%)"
        if lvl == estimate.CRITICAL:
            self.get_logger().error(
                f"{text}: CRITICAL, swap or charge the 4S Jetson pack now",
                throttle_duration_sec=5.0,
            )
        elif lvl == estimate.LOW:
            self.get_logger().warn(
                f"{text}: low, charge soon", throttle_duration_sec=30.0
            )
        elif lvl == estimate.OVERVOLTAGE:
            self.get_logger().warn(
                f"{text}: above 4.25 V per cell, check the cell count or voltage_scale",
                throttle_duration_sec=30.0,
            )
        elif lvl != self.last_level:
            self.get_logger().info(text)
        self.last_level = lvl

    def destroy_node(self) -> None:
        if hasattr(self.sensor, "close"):
            self.sensor.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = JetsonBatteryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
