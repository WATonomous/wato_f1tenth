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
"""The /hal contract: which checks apply, and how an observation is judged. No ROS imports."""

from dataclasses import dataclass, field

import yaml

PASS = "PASS"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"

# Clients that may still use old names without failing the check (stale Foxglove layouts)
TOLERATED_LEGACY_NODES = {"foxglove_bridge"}


@dataclass
class TopicCheck:
    name: str
    type: str
    group: str
    min_hz: float = 0.0
    frame_id: str = ""
    publishers: bool = False
    subscribers: bool = False


@dataclass
class TfCheck:
    parent: str
    child: str
    group: str


@dataclass
class Observation:
    """What the checker saw for one topic during the sampling window."""

    types: list = field(default_factory=list)
    publishers: list = field(default_factory=list)  # node names
    subscribers: list = field(default_factory=list)
    stamps: list = field(default_factory=list)  # receive times in seconds
    frame_ids: set = field(default_factory=set)
    frames_connected: dict = field(
        default_factory=dict
    )  # frame -> resolves to base_link

    @property
    def hz(self):
        if len(self.stamps) < 2 or self.stamps[-1] <= self.stamps[0]:
            return 0.0
        return (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0])


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_sources(
    platform, imu="auto", localization="auto", mapping=False, realsense=False
):
    """Mirror the launch files' auto rules: which IMU and localization source is running."""
    if imu == "auto":
        if platform == "sim":
            imu = "sim"  # the sim container relays AutoDRIVE's IMU
        else:
            imu = "realsense" if realsense else "none"
    if localization == "auto":
        localization = "pf" if platform == "car" else "truth"
    if mapping:
        localization = (
            "none"  # slam_toolbox maps; nothing publishes /hal/localization/odom
        )
    return {"imu": imu, "localization": localization}


def _active(entry, groups, sources):
    if groups and entry.get("group") not in groups:
        return False
    when = entry.get("when")
    return not when or sources.get(when, "none") != "none"


def topic_checks(contract, platform, groups, sources):
    checks = []
    for t in contract["topics"]:
        if not _active(t, groups, sources):
            continue
        checks.append(
            TopicCheck(
                name=t["name"],
                type=t["type"],
                group=t["group"],
                min_hz=float((t.get("min_hz") or {}).get(platform, 0.0)),
                frame_id=t.get("frame_id", ""),
                publishers=bool(t.get("publishers", False)),
                subscribers=bool(t.get("subscribers", False)),
            )
        )
    return checks


def tf_checks(contract, groups, sources):
    return [
        TfCheck(e["parent"], e["child"], e["group"])
        for e in contract.get("tf", [])
        if _active(e, groups, sources)
    ]


def judge_topic(check, obs):
    """Return (status, detail) for one topic."""
    if not obs.types:
        if check.publishers or check.subscribers or check.min_hz > 0:
            return FAIL, "not on the graph"
        return SKIP, "not on the graph (optional)"
    if check.type not in obs.types:
        return FAIL, f"type {', '.join(obs.types)}, expected {check.type}"
    if check.publishers and not obs.publishers:
        return FAIL, "no publisher"
    if check.subscribers and not obs.subscribers:
        return FAIL, "no subscriber"

    notes = []
    if check.min_hz > 0:
        if obs.hz < check.min_hz:
            return FAIL, f"{obs.hz:.1f} Hz, expected >= {check.min_hz:.0f} Hz"
        notes.append(f"{obs.hz:.1f} Hz")
    if check.frame_id:
        frames = sorted(obs.frame_ids)
        if not frames:
            return FAIL, "no messages to check the frame"
        if check.frame_id == "any":
            bad = [f for f in frames if not f or not obs.frames_connected.get(f)]
            if bad:
                return FAIL, f"frame {bad[0]!r} has no TF to base_link"
        elif frames != [check.frame_id]:
            return (
                FAIL,
                f"frame {', '.join(repr(f) for f in frames)}, expected {check.frame_id!r}",
            )
        notes.append(f"frame {frames[0]}")
    if not notes:
        notes.append(f"{len(obs.publishers)} pub / {len(obs.subscribers)} sub")
    return PASS, ", ".join(notes)


def judge_legacy(name, nodes):
    """nodes: every node publishing or subscribing to an old name that still exists."""
    if not nodes:
        return PASS, "absent"
    if set(nodes) <= TOLERATED_LEGACY_NODES:
        return (
            WARN,
            f"used only by {', '.join(sorted(set(nodes)))} (update your Foxglove layout to /hal/*)",
        )
    return FAIL, f"still used by {', '.join(sorted(set(nodes)))}"
