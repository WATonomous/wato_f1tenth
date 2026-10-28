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
"""Every controller and local planner is registered completely (agents/workflows/add-controller.md).

CONTROLLER=<name> and LOCAL_PLANNER=<name> pick launch/controllers/<name>.launch.yaml and
launch/local_planners/<name>.launch.yaml. These checks run without ROS.
"""

import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
KINDS = ("controllers", "local_planners")
INTERFACE_ARGS = {"platform", "config_dir", "local_planner"}
PLATFORMS = ("car", "sim")
# Topic parameters may only name contract topics, debug topics or the algorithm's own namespace
TOPIC_PREFIXES = ("/hal/", "/global_planner/", "/local_planner/", "/debug/")


def registered():
    return [
        (kind, f[: -len(".launch.yaml")])
        for kind in KINDS
        for f in sorted(os.listdir(os.path.join(PKG, "launch", kind)))
        if f.endswith(".launch.yaml")
    ]


def launch_text(kind, name):
    with open(os.path.join(PKG, "launch", kind, f"{name}.launch.yaml")) as f:
        return f.read()


def walk(node):
    """Every dict in a parsed launch file, including those nested in groups."""
    if isinstance(node, dict):
        yield node
        for v in node.values():
            yield from walk(v)
    elif isinstance(node, list):
        for v in node:
            yield from walk(v)


def exec_depends():
    root = ET.parse(os.path.join(PKG, "package.xml")).getroot()
    return {e.text for e in root.iter("exec_depend")}


def config_files(text):
    return sorted(set(re.findall(r"\$\(var config_dir\)/([\w.-]+\.yaml)", text)))


def test_none_exists_for_both_kinds():
    for kind in KINDS:
        assert (kind, "none") in registered()


@pytest.mark.parametrize("kind,name", registered())
def test_declares_the_interface_args(kind, name):
    spec = yaml.safe_load(launch_text(kind, name))
    args = {e["arg"]["name"] for e in spec["launch"] if "arg" in e}
    assert INTERFACE_ARGS <= args, f"{kind}/{name}: declare {INTERFACE_ARGS - args}"


@pytest.mark.parametrize("kind,name", registered())
def test_packages_are_exec_depends(kind, name):
    text = launch_text(kind, name)
    spec = yaml.safe_load(text)
    pkgs = {d["node"]["pkg"] for d in walk(spec) if isinstance(d.get("node"), dict)}
    pkgs |= set(re.findall(r"\$\(find-pkg-share ([\w-]+)\)", text))
    missing = pkgs - exec_depends() - {"action_bringup"}
    assert not missing, f"add <exec_depend> to action_bringup/package.xml: {missing}"


@pytest.mark.parametrize("kind,name", registered())
def test_config_for_both_platforms(kind, name):
    text = launch_text(kind, name)
    spec = yaml.safe_load(text)
    nodes = {d["node"]["name"] for d in walk(spec) if isinstance(d.get("node"), dict)}
    for platform in PLATFORMS:
        for f in config_files(text):
            path = os.path.join(PKG, "config", platform, f)
            assert os.path.isfile(path), f"{kind}/{name} needs config/{platform}/{f}"
            with open(path) as fh:
                keys = set(yaml.safe_load(fh) or {})
            # ROS silently ignores a file keyed by any other name
            assert keys <= nodes | {"/**"}, (
                f"config/{platform}/{f}: keys {keys}, nodes {nodes}"
            )


@pytest.mark.parametrize("kind,name", registered())
def test_topics_stay_in_their_namespaces(kind, name):
    for platform in PLATFORMS:
        for f in config_files(launch_text(kind, name)):
            with open(os.path.join(PKG, "config", platform, f)) as fh:
                spec = yaml.safe_load(fh) or {}
            for node in spec.values():
                for key, value in node.get("ros__parameters", {}).items():
                    if not key.endswith("_topic"):
                        continue
                    where = f"config/{platform}/{f}: {key}={value}"
                    assert value.startswith(TOPIC_PREFIXES + (f"/{name}/",)), where
                    if kind == "local_planners":
                        # only the controller drives; a planner publishes under /local_planner/
                        assert not value.startswith("/hal/drive/"), where
