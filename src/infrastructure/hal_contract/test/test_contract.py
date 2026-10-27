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
"""Unit tests for the contract logic (no ROS graph needed)."""

import os

import pytest

from hal_contract import contract as c

CONTRACT = c.load(
    os.path.join(os.path.dirname(__file__), "..", "config", "contract.yaml")
)


def obs(**kw):
    return c.Observation(**kw)


def names(checks):
    return [ch.name for ch in checks]


def test_every_contract_topic_is_under_its_namespace():
    namespaces = {
        "sensors": ("/hal/",),
        "drive": ("/hal/",),
        "planning": ("/global_planner/", "/local_planner/"),
    }
    for t in CONTRACT["topics"]:
        assert t["name"].startswith(namespaces[t["group"]])
        assert t["type"].count("/") == 2
    assert not {t["name"] for t in CONTRACT["topics"]} & set(CONTRACT["legacy"])


def test_controller_and_local_planner_checks():
    def planning(controller, local_planner):
        sources = {"controller": controller, "local_planner": local_planner}
        return {
            ch.name: ch
            for ch in c.topic_checks(CONTRACT, "car", {"drive", "planning"}, sources)
        }

    off = planning("none", "none")
    assert not off["/hal/drive/autonomy"].publishers
    assert off["/hal/drive/autonomy"].subscribers
    assert "/global_planner/path" not in off
    assert "/local_planner/path" not in off

    pp = planning("pure_pursuit", "none")
    assert pp["/hal/drive/autonomy"].publishers
    assert pp["/global_planner/path"].publishers
    assert "/local_planner/path" not in pp

    both = planning("pure_pursuit", "frenet")
    assert both["/local_planner/path"].frame_id == "base_link"
    assert both["/local_planner/path"].subscribers
    assert both["/local_planner/active"].publishers
    assert not planning("none", "frenet")["/local_planner/path"].subscribers


@pytest.mark.parametrize(
    "platform,kw,expected",
    [
        ("car", {}, {"imu": "none", "localization": "pf", "battery": "vesc"}),
        (
            "car",
            {"realsense": True},
            {"imu": "realsense", "localization": "pf", "battery": "vesc"},
        ),
        (
            "car",
            {"imu": "vesc"},
            {"imu": "vesc", "localization": "pf", "battery": "vesc"},
        ),
        ("sim", {}, {"imu": "sim", "localization": "truth", "battery": "none"}),
        (
            "sim",
            {"localization": "pf"},
            {"imu": "sim", "localization": "pf", "battery": "none"},
        ),
        (
            "sim",
            {"mapping": True},
            {"imu": "sim", "localization": "none", "battery": "none"},
        ),
    ],
)
def test_resolve_sources_mirrors_launch_auto_rules(platform, kw, expected):
    assert c.resolve_sources(platform, **kw) == expected


def test_inactive_sources_and_groups_are_skipped():
    sources = c.resolve_sources("car", localization="none")
    checks = names(c.topic_checks(CONTRACT, "car", {"sensors"}, sources))
    assert checks == ["/hal/scan", "/hal/odom", "/hal/battery"]
    assert [
        (t.parent, t.child) for t in c.tf_checks(CONTRACT, {"sensors"}, sources)
    ] == [
        ("odom", "base_link"),
        ("base_link", "laser"),
    ]


def test_rates_are_per_platform():
    sources = c.resolve_sources("sim")
    scan = c.topic_checks(CONTRACT, "sim", {"sensors"}, sources)[0]
    assert scan.min_hz == 15.0


SCAN = c.TopicCheck(
    "/hal/scan",
    "sensor_msgs/msg/LaserScan",
    "sensors",
    min_hz=15.0,
    frame_id="laser",
    publishers=True,
)


def test_scan_passes_at_rate_with_frame():
    o = obs(
        types=[SCAN.type],
        publishers=["urg_node"],
        stamps=[i * 0.05 for i in range(21)],
        frame_ids={"laser"},
    )
    assert c.judge_topic(SCAN, o)[0] == c.PASS


@pytest.mark.parametrize(
    "o,detail",
    [
        (obs(), "not on the graph"),
        (
            obs(types=["sensor_msgs/msg/PointCloud2"], publishers=["x"]),
            "expected sensor_msgs/msg/LaserScan",
        ),
        (obs(types=[SCAN.type]), "no publisher"),
        (
            obs(
                types=[SCAN.type],
                publishers=["x"],
                stamps=[0.0, 1.0],
                frame_ids={"laser"},
            ),
            "expected >= 15 Hz",
        ),
        (
            obs(
                types=[SCAN.type],
                publishers=["x"],
                stamps=[i * 0.05 for i in range(21)],
                frame_ids={"lidar"},
            ),
            "'lidar'",
        ),
    ],
)
def test_scan_failures(o, detail):
    status, text = c.judge_topic(SCAN, o)
    assert status == c.FAIL
    assert detail in text


def test_any_frame_must_connect_to_base_link():
    imu = c.TopicCheck(
        "/hal/imu", "sensor_msgs/msg/Imu", "sensors", frame_id="any", publishers=True
    )
    base = {
        "types": [imu.type],
        "publishers": ["vesc_driver_node"],
        "stamps": [0.0, 0.01],
    }
    assert c.judge_topic(imu, obs(frame_ids={""}, **base))[0] == c.FAIL
    assert (
        c.judge_topic(
            imu,
            obs(frame_ids={"imu_link"}, frames_connected={"imu_link": True}, **base),
        )[0]
        == c.PASS
    )


def test_optional_drive_topic_without_rate():
    autonomy = c.TopicCheck(
        "/hal/drive/autonomy",
        "ackermann_msgs/msg/AckermannDriveStamped",
        "drive",
        subscribers=True,
    )
    assert (
        c.judge_topic(
            autonomy, obs(types=[autonomy.type], subscribers=["ackermann_mux"])
        )[0]
        == c.PASS
    )
    assert (
        c.judge_topic(
            autonomy, obs(types=[autonomy.type], publishers=["pure_pursuit_node"])
        )[0]
        == c.FAIL
    )


def test_legacy_names():
    assert c.judge_legacy("/scan", [])[0] == c.PASS
    assert c.judge_legacy("/dead_man_switch", ["foxglove_bridge"])[0] == c.WARN
    assert c.judge_legacy("/odom", ["vesc_to_odom_node"])[0] == c.FAIL
