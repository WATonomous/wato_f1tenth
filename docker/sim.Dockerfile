ARG BASE_IMAGE=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
# Built locally with `./watod build-base` until prebuilt images are published.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code
COPY src/sim sim
COPY src/wato_test wato_test
# Contract checker for sim_bringup's bag replay test
COPY src/infrastructure/hal_contract hal_contract

################################# Dependencies ################################
# NOTE: You should be relying on ROSDEP as much as possible
# Use this stage as a last resort
FROM ${BASE_IMAGE} AS dependencies

# The devkit's package.xml only declares rclpy, so rosdep can't see these.
RUN apt-get update && apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-tf2-ros-py" \
    python3-numpy \
    python3-pil \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*

# Socket.IO server stack the simulator talks to. python-socketio 4.2 /
# python-engineio 3.13 match the simulator's protocol (devkit
# requirements_python_3.10.txt); gevent is newer than the devkit's pin
# because 21.12 does not build on Python 3.12. attrdict3 is the maintained
# fork of attrdict (imported by the devkit's config.py) that works on 3.12.
RUN pip3 install --no-cache-dir \
    "python-socketio==4.2.0" \
    "python-engineio==3.13.0" \
    "gevent==24.11.1" \
    "gevent-websocket==0.10.1" \
    "attrdict3==2.0.2"

# AutoDRIVE devkit: the ROS 2 bridge the simulator connects to on port 4567.
# Kept in dependencies (pinned release, only declares rclpy in package.xml) so both
# build and develop (sim_bringup_dev) inherit it without coupling develop to src/sim edits.
# Its upstream lint tests are dropped so `watod test sim` only tests our code.
ARG AUTODRIVE_DEVKIT_URL=https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-icra/autodrive_devkit.zip
ARG AUTODRIVE_DEVKIT_SHA256=ac2c3bb49bb32b7c84e000ddc7fecf07e191d3d03d00113d60dc4da693941ee0
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN mkdir -p "${AMENT_WS}/src" && \
    curl -fsSL -o /tmp/autodrive_devkit.zip "${AUTODRIVE_DEVKIT_URL}" && \
    echo "${AUTODRIVE_DEVKIT_SHA256}  /tmp/autodrive_devkit.zip" | sha256sum -c - && \
    python3 -m zipfile -e /tmp/autodrive_devkit.zip "${AMENT_WS}/src" && \
    rm -rf "${AMENT_WS}/src/autodrive_devkit/test" /tmp/autodrive_devkit.zip
