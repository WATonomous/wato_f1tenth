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
"""System monitor: computer health on /system/status, pack power on /system/power, black box.

/system/status (1 Hz): CPU, GPU, memory, disk, temperatures and, on the Jetson, its INA3221 power
rails. /system/power (10 Hz, when there is a VESC or a power monitor): pack voltage and drive current
from the VESC, the Jetson's module power, and an estimate of the total drawn from the pack. With
blackbox_dir set (the car), the same numbers go to a CSV that survives a sudden power cut.
"""

import math
import socket

import rclpy
from f1tenth_msgs.msg import PowerRail, PowerStatus, SystemStatus, Temperature
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vesc_msgs.msg import VescStateStamped

from system_monitor import readers
from system_monitor.blackbox import BlackBox

NAN = float("nan")


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        self.declare_parameter("status_topic", "/system/status")
        self.declare_parameter("power_topic", "/system/power")
        self.declare_parameter("vesc_topic", "/sensors/core")
        self.declare_parameter("status_hz", 1.0)
        self.declare_parameter("power_hz", 10.0)
        self.declare_parameter("sysfs_root", "/")
        self.declare_parameter("temperature_alarm_c", 85.0)
        # Power-board loads the Jetson doesn't measure (lidar, 12 V / 5 V outputs), and the
        # efficiency of the board's converters; set per platform in YAML
        self.declare_parameter("other_loads_w", 0.0)
        self.declare_parameter("converter_efficiency", 1.0)
        self.declare_parameter("vesc_stale_s", 1.0)
        # Black box: empty = off
        self.declare_parameter("blackbox_dir", "")
        self.declare_parameter("blackbox_keep_files", 30)

        p = self.get_parameter
        self.root = p("sysfs_root").value
        self.temp_alarm = p("temperature_alarm_c").value
        self.other_loads_w = p("other_loads_w").value
        self.efficiency = p("converter_efficiency").value
        self.vesc_stale_s = p("vesc_stale_s").value

        self.hostname = socket.gethostname()
        self.prev_cpu = readers.cpu_times(self.root)
        self.cpu = [NAN]
        self.ram = (NAN, NAN)
        self.gpu = NAN
        self.temps = []
        self.rails = []
        self.alarms = []
        self.pack_v = NAN
        self.drive_a = NAN
        self.vesc_rx = None

        self.status_pub = self.create_publisher(
            SystemStatus, p("status_topic").value, 10
        )
        self.power_pub = self.create_publisher(PowerStatus, p("power_topic").value, 10)
        self.create_subscription(
            VescStateStamped, p("vesc_topic").value, self.on_vesc, 10
        )
        self.create_timer(
            1.0 / p("status_hz").value, lambda: self.guarded(self.publish_status)
        )
        self.create_timer(
            1.0 / p("power_hz").value, lambda: self.guarded(self.publish_power)
        )

        self.blackbox = None
        if p("blackbox_dir").value:
            self.blackbox = BlackBox(
                p("blackbox_dir").value, p("blackbox_keep_files").value
            )
            self.get_logger().info(f"black box: {self.blackbox.path}")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def guarded(self, step):
        """Run a timer step; log a failure instead of letting it kill the node (and the black box)."""
        try:
            step()
        except Exception as e:  # noqa: BLE001 - the monitor must outlive any one bad reading
            self.get_logger().error(
                f"{step.__name__} failed: {e!r}", throttle_duration_sec=10.0
            )

    def on_vesc(self, msg):
        self.pack_v = msg.state.voltage_input
        self.drive_a = msg.state.current_input
        self.vesc_rx = self.now_s()

    def publish_status(self):
        cpu_now = readers.cpu_times(self.root)
        self.cpu = readers.cpu_percent(self.prev_cpu, cpu_now) or [NAN]
        self.prev_cpu = cpu_now
        self.ram = readers.memory(self.root)
        self.gpu = readers.gpu_percent(self.root)
        self.temps = readers.temperatures(self.root)
        self.rails = readers.power_rails(self.root)

        alarms = [
            f"{name} current alarm" for name, _, _, _, alarm in self.rails if alarm
        ]
        alarms += [
            f"{name} at {c:.0f} C" for name, c in self.temps if c >= self.temp_alarm
        ]
        if alarms and alarms != self.alarms:
            self.get_logger().warn("; ".join(alarms))
        self.alarms = alarms

        msg = SystemStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hostname = self.hostname
        msg.cpu_percent = float(self.cpu[0])
        msg.cpu_core_percent = [float(c) for c in self.cpu[1:]]
        msg.cpu_freq_mhz = float(readers.cpu_freq_mhz(self.root))
        msg.gpu_percent = float(self.gpu)
        msg.ram_used_percent, msg.ram_used_mb = (float(v) for v in self.ram)
        msg.disk_used_percent = float(readers.disk_used_percent("/"))
        msg.temperatures = [
            Temperature(name=n, celsius=float(c)) for n, c in self.temps
        ]
        msg.max_temperature_c = float(max((c for _, c in self.temps), default=NAN))
        msg.power_rails = [
            PowerRail(name=n, voltage_v=float(v), current_a=float(a), power_w=float(w))
            for n, v, a, w, _ in self.rails
        ]
        msg.module_power_w = float(readers.module_power_w(self.rails))
        msg.alarms = alarms
        self.status_pub.publish(msg)

    def publish_power(self):
        # The power monitor is cheap to read: refresh it at this rate for the black box
        rails = readers.power_rails(self.root) if self.rails else []
        compute_w = readers.module_power_w(rails)
        fresh = (
            self.vesc_rx is not None and self.now_s() - self.vesc_rx < self.vesc_stale_s
        )
        pack_v = self.pack_v if fresh else NAN
        drive_a = self.drive_a if fresh else NAN
        if not fresh and math.isnan(compute_w):
            return  # nothing to report (the sim, or a PC)

        drive_w = pack_v * drive_a
        total_w = readers.total_power(
            drive_w, compute_w, self.other_loads_w, self.efficiency
        )
        msg = PowerStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pack_voltage_v = float(pack_v)
        msg.drive_current_a = float(drive_a)
        msg.drive_power_w = float(drive_w)
        msg.compute_power_w = float(compute_w)
        msg.total_power_w = float(total_w)
        msg.total_current_a = float(total_w / pack_v) if fresh and pack_v > 0 else NAN
        self.power_pub.publish(msg)

        if self.blackbox:
            self.blackbox.write(
                {
                    "pack_voltage_v": msg.pack_voltage_v,
                    "drive_current_a": msg.drive_current_a,
                    "drive_power_w": msg.drive_power_w,
                    "compute_power_w": msg.compute_power_w,
                    "total_power_w": msg.total_power_w,
                    "cpu_percent": float(self.cpu[0]),
                    "ram_used_percent": float(self.ram[0]),
                    "max_temperature_c": float(
                        max((c for _, c in self.temps), default=NAN)
                    ),
                    "gpu_percent": float(self.gpu),
                    "alarms": "; ".join(self.alarms),
                }
            )

    def destroy_node(self):
        if self.blackbox:
            self.blackbox.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):  # Ctrl-C, or docker stop (SIGTERM)
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
