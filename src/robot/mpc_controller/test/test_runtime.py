#!/usr/bin/env python3
"""ROS integration checks. Run after sourcing the built workspace."""
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

os.environ.setdefault('ROS_DOMAIN_ID', str(120 + os.getpid() % 90))
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class Runtime(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('mpc_runtime_test')
        self.processes = []
        self.logs = []
        self.commands = []
        self.reasons = []
        self.grids = []
        self.metrics = []
        self.node.create_subscription(Float64MultiArray, '/mpc/metrics',
                                      lambda m: self.metrics.append(m.data), 100)
        self.heartbeat = None
        self.manual = None
        self.emergency = False
        self.clock_enabled = False
        self.clock_paused = False
        self.clock_offset = 0
        self.clock_value = None
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 1)
        self.node.create_timer(.01, self.publish_clock)
        self.node.create_subscription(AckermannDriveStamped, '/ackermann_cmd',
                                      lambda m: self.commands.append((time.monotonic(), m)), 100)
        self.node.create_subscription(String, '/mpc/status',
                                      lambda m: self.reasons.append(m.data), 100)
        self.enable_pub = self.node.create_publisher(Bool, '/dead_man_switch', 1)
        self.manual_pub = self.node.create_publisher(AckermannDriveStamped, '/drive/keyboard', 1)
        self.emergency_pub = self.node.create_publisher(AckermannDriveStamped, '/drive/ebreak', 1)
        self.node.create_timer(.04, self.publish_inputs)

    def tearDown(self):
        for process in reversed(self.processes):
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in self.logs:
            log.seek(0)
            text = log.read()
            if text and any(p.returncode not in (0, -15) for p in self.processes):
                print(text[-2500:])
            log.close()
        self.node.destroy_node()
        rclpy.shutdown()

    def start(self, package, executable, *arguments):
        log = tempfile.TemporaryFile(mode='w+')
        binary = Path(get_package_prefix(package)) / 'lib' / package / executable
        process = subprocess.Popen([str(binary), *arguments], stdout=log, stderr=log)
        self.processes.append(process)
        self.logs.append(log)

    def stack(self, stall=0, simulated_clock=False, visuals=True):
        self.clock_enabled = simulated_clock
        config = Path(get_package_share_directory('mpc_controller')) / 'config' / 'mux.yaml'
        self.start('ackermann_mux', 'ackermann_mux', '--ros-args', '--params-file', str(config))
        self.start('mpc_controller', 'mpc_node', '--ros-args', '-p', f'test_solver_stall_ms:={stall}',
                   '-p', f'publish_visualization:={str(visuals).lower()}',
                   '-p', f'use_sim_time:={str(simulated_clock).lower()}')
        self.start('mpc_controller', 'fixture_simulator', '--ros-args',
                   '-p', f'use_sim_time:={str(simulated_clock).lower()}')
        self.pump(1.)
        self.assertTrue(all(p.poll() is None for p in self.processes), 'A ROS process exited')

    def publish_clock(self):
        if self.clock_enabled:
            if not self.clock_paused or self.clock_value is None:
                self.clock_value = self.node.get_clock().now().to_msg()
                self.clock_value.sec -= self.clock_offset
            self.clock_pub.publish(Clock(clock=self.clock_value))

    def publish_inputs(self):
        if self.heartbeat is not None:
            self.enable_pub.publish(Bool(data=self.heartbeat))
        for active, publisher, speed in ((self.manual is not None, self.manual_pub, self.manual),
                                         (self.emergency, self.emergency_pub, 0.)):
            if active:
                message = AckermannDriveStamped()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.drive.speed = speed
                publisher.publish(message)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)

    def stopped(self):
        recent = [m for t, m in self.commands if t > time.monotonic() - .08]
        self.assertTrue(recent, 'No recent selected commands')
        self.assertTrue(all(m.drive.speed == 0 for m in recent), 'Moving command after stop gate')

    def moving(self):
        recent = [m for t, m in self.commands if t > time.monotonic() - .5]
        self.assertTrue(any(m.drive.speed > .1 for m in recent), 'No accepted moving commands')

    def parameter(self, name, value):
        client = AsyncParameterClient(self.node, '/mpc_fixture_simulator')
        self.assertTrue(client.wait_for_services(timeout_sec=2))
        future = client.set_parameters([Parameter(name, value=value)])
        end = time.monotonic() + 2
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.01)
        self.assertTrue(future.done())
        self.assertTrue(future.result().results[0].successful)

    def test_enable_freshness_and_arbitration(self):
        self.stack()
        self.stopped()
        self.heartbeat = True
        self.metrics.clear()
        self.pump(2.)
        self.moving()
        times = sorted(m[0] for m in self.metrics)
        approved = sum(m[4] for m in self.metrics)
        rss = 0
        for process in self.processes:
            for line in Path(f'/proc/{process.pid}/status').read_text().splitlines():
                if line.startswith('VmRSS:'):
                    rss += int(line.split()[1])
        print(f'ROS solo samples={len(times)} approved={int(approved)} '
              f'p50_ms={times[len(times)//2]:.3f} p99_ms={times[int(.99*(len(times)-1))]:.3f} '
              f'max_ms={times[-1]:.3f} node_mux_fixture_rss_KiB={rss}')
        self.heartbeat = None
        self.pump(.35)
        self.stopped()
        self.assertIn('EnableTimeout', self.reasons)
        self.heartbeat = True
        self.pump(1.)
        self.moving()
        self.parameter('publish_state', False)
        self.pump(.22)
        self.stopped()
        self.parameter('publish_state', True)
        self.pump(1.)
        self.moving()
        self.parameter('publish_grid', False)
        self.pump(.4)
        self.stopped()
        self.parameter('publish_grid', True)
        self.pump(1.)
        self.moving()
        self.manual = .23
        self.pump(.35)
        recent = [m for t, m in self.commands if t > time.monotonic() - .08]
        self.assertTrue(recent)
        self.assertTrue(all(abs(m.drive.speed - .23) < 1e-6 for m in recent))
        self.emergency = True
        self.pump(.3)
        self.stopped()
        self.emergency = False
        self.manual = None
        self.pump(1.2)
        self.moving()
        self.heartbeat = False
        self.pump(.2)
        self.stopped()

    def test_paused_and_rewound_simulation_clock(self):
        self.stack(simulated_clock=True)
        self.heartbeat = True
        self.pump(2.)
        self.moving()
        self.clock_paused = True
        self.pump(.4)
        self.stopped()
        self.clock_paused = False
        self.pump(1.)
        self.moving()
        self.clock_offset = 10
        self.pump(1.5)
        self.assertIn('ClockReset', self.reasons)
        self.moving()

    def test_stalled_solver_cannot_publish_motion(self):
        self.stack(stall=80)
        self.heartbeat = True
        self.pump(1.)
        self.stopped()
        self.assertIn('Deadline', self.reasons)

    def test_foxglove_topics_and_stale_visual_clearing(self):
        tracks, markers, search, paths, grids = [], [], [], [], []
        dynamic = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(MarkerArray, '/mpc/visualization/cycle', markers.append, dynamic)
        self.node.create_subscription(OccupancyGrid, '/mpc/visualization/search_space', search.append, dynamic)
        self.node.create_subscription(RosPath, '/mpc/predicted_path', paths.append, dynamic)
        self.node.create_subscription(OccupancyGrid, '/costmap', grids.append, 10)
        self.stack()
        # A late Foxglove connection must still receive static geometry.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(MarkerArray, '/mpc/visualization/track', tracks.append, latched)
        self.pump(.4)
        self.assertTrue(tracks and markers and search and paths and grids)
        raceline = next(m for m in tracks[-1].markers if m.ns == 'raceline')
        self.assertGreater(len(raceline.points), 100)
        self.assertTrue(all(p.z < .1 for p in raceline.points))
        self.assertEqual(raceline.header.frame_id, 'map')
        self.assertFalse(paths[-1].poses, 'Disabled path must be empty')
        self.assertTrue(any(0 in m.data for m in search), 'No legal search cells drawn')
        observed_stamps = [g.header.stamp for g in grids]
        self.assertTrue(all(m.header.stamp in observed_stamps for m in search if len(m.data) > 1))
        self.heartbeat = True
        self.pump(1.5)
        self.moving()
        self.assertTrue(any(len(p.poses) > 1 for p in paths))
        names = {m.ns for array in markers for m in array.markers}
        self.assertTrue({'approved_proposal', 'vehicle_footprint', 'covering_circle_plus_clearance',
                         'seed_route_rear_axle', 'solver_corridors_covering_center', 'command_status'} <= names)
        self.parameter('publish_state', False)
        self.parameter('publish_grid', False)
        self.pump(.55)
        self.stopped()
        self.assertFalse(paths[-1].poses, 'Old approved path remained after stop')
        self.assertEqual(list(search[-1].data), [-1], 'Old legal-space overlay remained')
        latest_names = {m.ns for m in markers[-1].markers}
        self.assertFalse({'approved_proposal', 'unapproved_proposal', 'vehicle_footprint'} & latest_names)
        self.assertEqual(markers[-1].markers[0].action, 3)

    def test_foxglove_visualizations_can_be_disabled(self):
        self.stack(visuals=False)
        self.assertEqual(self.node.count_publishers('/mpc/visualization/cycle'), 0)
        self.assertEqual(self.node.count_publishers('/mpc/visualization/search_space'), 0)

    def test_simulator_bridge_clears_a_stale_throttle_request(self):
        self.start('sim_util', 'ackermann_converter_node')
        throttle = []
        self.node.create_subscription(Float32, '/autodrive/roboracer_1/throttle_command',
                                      lambda m: throttle.append(m.data), 10)
        publisher = self.node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
        self.pump(.6)
        command = AckermannDriveStamped()
        command.drive.speed = .5
        publisher.publish(command)
        self.pump(.04)
        self.assertTrue(any(v > 0 for v in throttle))
        self.pump(.15)
        self.assertEqual(throttle[-1], 0.)
        command.drive.speed = float('nan')
        publisher.publish(command)
        self.pump(.04)
        self.assertEqual(throttle[-1], 0.)

    def test_costmap_preserves_scan_time_and_invalid_rays(self):
        self.start('costmap', 'costmap_node', '--ros-args', '-r', '/scan:=/test_scan',
                   '-r', '/costmap:=/test_costmap')
        self.node.create_subscription(OccupancyGrid, '/test_costmap', self.grids.append, 10)
        publisher = self.node.create_publisher(LaserScan, '/test_scan', 1)
        self.pump(.6)
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.header.stamp = self.node.get_clock().now().to_msg()
        scan.header.stamp.sec -= 1
        scan.angle_min = 0.
        scan.angle_increment = .1
        scan.range_min = .1
        scan.range_max = 3.
        scan.ranges = [float('nan'), float('-inf'), float('inf'), 1.2]
        publisher.publish(scan)
        self.pump(.3)
        self.assertTrue(self.grids)
        grid = self.grids[-1]
        self.assertEqual(grid.header.stamp, scan.header.stamp)
        self.assertEqual(grid.header.frame_id, 'base_link')
        self.assertIn(100, grid.data)
        self.assertIn(0, grid.data)
        self.assertIn(-1, grid.data)
        count = len(self.grids)
        scan.angle_min = float('nan')
        publisher.publish(scan)
        self.pump(.2)
        self.assertEqual(len(self.grids), count, 'Malformed scan metadata was accepted')


if __name__ == '__main__':
    unittest.main()
