#!/usr/bin/env python3
"""Bounded live Porto trial: enable heartbeat, record telemetry, then verify stopping."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, Int32, String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('autodrive-trial.json'))
    parser.add_argument('--require-laps', type=int, default=0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600:
        parser.error('--seconds must be in [1, 600]')
    if not 0 <= args.require_laps <= 10:
        parser.error('--require-laps must be in [0, 10]')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('autodrive_trial')
    enable = node.create_publisher(Bool, '/dead_man_switch', 1)
    active = False
    node.create_timer(.04, lambda: enable.publish(Bool(data=active)))
    reasons, metrics, odometry, commands = Counter(), [], [], []
    counters = {}
    node.create_subscription(String, '/mpc/status', lambda m: reasons.update([m.data]), 100)
    node.create_subscription(Float64MultiArray, '/mpc/metrics', lambda m: metrics.append(list(m.data)), 1000)
    node.create_subscription(Int32, '/autodrive/roboracer_1/collision_count',
                             lambda m: counters.update(collisions=m.data), 10)
    node.create_subscription(Int32, '/autodrive/roboracer_1/lap_count',
                             lambda m: counters.update(laps=m.data), 10)

    def state(m):
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        odometry.append([time.monotonic(), m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                         p.x, p.y, yaw, math.hypot(v.x, v.y)])

    node.create_subscription(Odometry, '/autodrive/roboracer_1/odom', state, 1000)
    node.create_subscription(AckermannDriveStamped, '/ackermann_cmd',
                             lambda m: commands.append([time.monotonic(), m.drive.speed,
                                                         m.drive.steering_angle]), 1000)

    def pump(seconds):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    failure = None
    initial = {}
    stop_time = None
    start = time.monotonic()
    stopped_after = None
    speed_at_disable = None
    try:
        discovery_deadline = time.monotonic()+10
        while time.monotonic() < discovery_deadline:
            pump(.05)
            if odometry and commands and 'collisions' in counters and 'laps' in counters:
                break
        if not odometry or not commands or 'collisions' not in counters or 'laps' not in counters:
            raise RuntimeError('Missing native odometry, selected commands, or collision telemetry')
        initial = counters.copy()
        reasons.clear()
        metrics.clear()
        active = True
        start = time.monotonic()
        next_report = start+15
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            pump(.02)
            if counters['collisions'] != initial['collisions']:
                raise RuntimeError('Simulator collision counter increased')
            if time.monotonic()-odometry[-1][0] > .5:
                raise RuntimeError('Native odometry stopped')
            if time.monotonic()-start > 12:
                recent = [s for s in odometry if s[0] > time.monotonic()-10]
                distance = sum(math.hypot(b[2]-a[2], b[3]-a[3]) for a, b in zip(recent, recent[1:]))
                if distance < .02:
                    raise RuntimeError('Vehicle made no progress for 10 seconds')
            if time.monotonic() >= next_report:
                print(f"elapsed={time.monotonic()-start:.1f}s pose={odometry[-1][2:5]} "
                      f"speed={odometry[-1][5]:.3f} counters={counters} statuses={dict(reasons)}", flush=True)
                next_report += 15
    except (RuntimeError, KeyboardInterrupt) as error:
        failure = str(error) or 'Interrupted'
    finally:
        active = False
        if rclpy.ok():
            speed_at_disable = odometry[-1][5] if odometry else None
            enable.publish(Bool(data=False))
            stop_time = time.monotonic()
            pump(4)
            stationary = [s for s in odometry if s[0] >= stop_time and s[5] <= .02]
            if stationary:
                stopped_after = stationary[0][0]-stop_time
            else:
                failure = failure or 'Measured speed did not fall below 0.02 m/s within 4 s'
        approved = [m for m in metrics if m[4] == 1]
        times = np.array([m[0] for m in metrics])
        travel = sum(math.hypot(b[2]-a[2], b[3]-a[3]) for a, b in zip(odometry, odometry[1:]))
        if not approved or travel < .2:
            failure = failure or 'No validated movement'
        final = [s for s in odometry if stop_time is not None and s[0] > stop_time+3.5]
        if not final or any(s[5] > .02 for s in final):
            failure = failure or 'Vehicle did not remain stopped after disable'
        if initial and counters['collisions'] != initial['collisions']:
            failure = failure or 'Simulator collision counter increased'
        if counters.get('laps', 0)-initial.get('laps', 0) < args.require_laps:
            failure = failure or 'Required simulator laps were not completed'
        moving = [s for s in odometry if start <= s[0] and (stop_time is None or s[0] < stop_time)]
        recent = [s for s in moving if s[0] >= (stop_time or time.monotonic())-10]
        recent_travel = sum(math.hypot(b[2]-a[2], b[3]-a[3]) for a, b in zip(recent, recent[1:]))
        if args.seconds >= 10 and recent_travel < .1:
            failure = failure or 'No progress in the final 10 seconds'
        cadence = np.diff([s[1] for s in moving])
        summary = {'failure': failure, 'initial_counters': initial, 'final_counters': counters,
                   'travel_m': travel, 'approved_cycles': len(approved), 'cycles': len(metrics),
                   'cycle_ms_p50_p95_p99_max': np.percentile(times, [50, 95, 99, 100]).tolist() if len(times) else [],
                   'deadline_misses_in_metrics': int(sum(times > 20)), 'status_counts': reasons,
                   'min_validated_clearance_m': min((m[5] for m in approved), default=None),
                   'native_odom_period_ms_p50_p99': (np.percentile(cadence, [50, 99])*1000).tolist() if len(cadence) else [],
                   'stopped_after_disable_s': stopped_after,
                   'speed_at_disable_mps': speed_at_disable,
                   'final_measured_speed_mps': odometry[-1][5] if odometry else None}
        serial_metrics = [[value if math.isfinite(value) else None for value in m] for m in metrics]
        args.output.write_text(json.dumps({'summary': summary, 'metrics': serial_metrics,
                                          'odometry': odometry, 'commands': commands}, indent=2,
                                         allow_nan=False))
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failure or not approved or travel < .2:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
