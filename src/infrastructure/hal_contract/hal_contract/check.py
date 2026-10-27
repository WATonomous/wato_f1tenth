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
"""Check the live ROS graph against the /hal contract. Exit code 1 if any check fails.

ros2 run hal_contract check --platform car --imu auto --localization auto
"""

import argparse
import os
import sys
import time

import rclpy
import rclpy.time
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message

from hal_contract import contract as c

NODE_NAME = "hal_contract_check"


def as_bool(value):
    return str(value).lower() in ("1", "true", "yes")


class Checker(Node):
    def __init__(self, checks):
        super().__init__(NODE_NAME)
        self.checks = checks
        self.obs = {ch.name: c.Observation() for ch in checks}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.subs = []

    def subscribe(self):
        # BEST_EFFORT + VOLATILE matches every publisher QoS, so the check never changes what it measures
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for ch in self.checks:
            if ch.min_hz <= 0 and not ch.frame_id:
                continue
            self.subs.append(
                self.create_subscription(
                    get_message(ch.type),
                    ch.name,
                    lambda m, n=ch.name: self.on_msg(n, m),
                    qos,
                )
            )

    def on_msg(self, name, msg):
        obs = self.obs[name]
        obs.stamps.append(time.monotonic())
        header = getattr(msg, "header", None)
        if header is not None:
            obs.frame_ids.add(header.frame_id)

    def endpoints(self, topic):
        pubs = [
            i.node_name
            for i in self.get_publishers_info_by_topic(topic)
            if i.node_name != NODE_NAME
        ]
        subs = [
            i.node_name
            for i in self.get_subscriptions_info_by_topic(topic)
            if i.node_name != NODE_NAME
        ]
        return pubs, subs

    def read_graph(self):
        types = dict(self.get_topic_names_and_types())
        for ch in self.checks:
            obs = self.obs[ch.name]
            obs.types = types.get(ch.name, [])
            obs.publishers, obs.subscribers = self.endpoints(ch.name)
            for frame in obs.frame_ids:
                obs.frames_connected[frame] = bool(frame) and self.can_transform(
                    "base_link", frame
                )
        return types

    def can_transform(self, target, source):
        return self.tf_buffer.can_transform(target, source, rclpy.time.Time())


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--platform", default=os.environ.get("PLATFORM", "sim"), choices=["car", "sim"]
    )
    parser.add_argument("--imu", default=os.environ.get("IMU", "auto"))
    parser.add_argument(
        "--localization", default=os.environ.get("LOCALIZATION", "auto")
    )
    parser.add_argument("--mapping", default=os.environ.get("MAPPING", "false"))
    parser.add_argument("--realsense", default=os.environ.get("REALSENSE", "false"))
    parser.add_argument(
        "--group",
        action="append",
        choices=["sensors", "drive"],
        help="only these groups",
    )
    parser.add_argument(
        "--warmup", type=float, default=2.0, help="seconds for discovery and TF"
    )
    parser.add_argument(
        "--duration", type=float, default=5.0, help="seconds of rate sampling"
    )
    parser.add_argument(
        "--contract",
        default=os.path.join(
            get_package_share_directory("hal_contract"), "config", "contract.yaml"
        ),
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    spec = c.load(args.contract)
    sources = c.resolve_sources(
        args.platform,
        args.imu,
        args.localization,
        as_bool(args.mapping),
        as_bool(args.realsense),
    )
    groups = set(args.group or [])
    checks = c.topic_checks(spec, args.platform, groups, sources)
    tfs = c.tf_checks(spec, groups, sources)

    rclpy.init()
    node = Checker(checks)
    print(
        f"/hal contract check: platform={args.platform} imu={sources['imu']} "
        f"localization={sources['localization']} groups={','.join(sorted(groups)) or 'all'}"
    )
    spin_for(node, args.warmup)
    node.subscribe()
    spin_for(node, args.duration)
    graph = node.read_graph()

    results = [(ch.name, *c.judge_topic(ch, node.obs[ch.name])) for ch in checks]
    for t in tfs:
        ok = node.can_transform(t.parent, t.child)
        results.append(
            (
                f"tf {t.parent} -> {t.child}",
                c.PASS if ok else c.FAIL,
                "resolves" if ok else "missing",
            )
        )
    for name in spec.get("legacy", []):
        nodes = []
        if name in graph:
            pubs, subs = node.endpoints(name)
            nodes = pubs + subs
        status, detail = c.judge_legacy(name, nodes)
        if status != c.PASS:
            results.append((f"legacy {name}", status, detail))

    width = max(len(r[0]) for r in results)
    for name, status, detail in results:
        print(f"  {status:4}  {name:{width}}  {detail}")
    failed = sum(1 for r in results if r[1] == c.FAIL)
    print(
        f"{len(results) - failed}/{len(results)} passed"
        + (f", {failed} FAILED" if failed else "")
    )

    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
