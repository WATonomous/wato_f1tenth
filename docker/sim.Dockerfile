ARG BASE_IMAGE=ghcr.io/watonomous/wato_f1tenth/base:jazzy-ubuntu24.04
# Built locally with `./watod build-base` until prebuilt images are published.

################################ Source ################################
# NOTE: You should add in the source stage in the following order:
#   - clone git repositories -> copy source code
# This will make your builds significantly faster
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# AutoDRIVE devkit: the ROS 2 bridge the simulator connects to on port 4567.
# Pinned to the 2026-ICRA release; identical to the devkit inside
# autodriveecosystem/autodrive_roboracer_api:2026-icra-practice. Its upstream
# lint tests are dropped so `watod test sim` only tests our code.
ARG AUTODRIVE_DEVKIT_URL=https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/download/2026-icra/autodrive_devkit.zip
ARG AUTODRIVE_DEVKIT_SHA256=ac2c3bb49bb32b7c84e000ddc7fecf07e191d3d03d00113d60dc4da693941ee0
RUN curl -fsSL -o /tmp/autodrive_devkit.zip "${AUTODRIVE_DEVKIT_URL}" && \
    echo "${AUTODRIVE_DEVKIT_SHA256}  /tmp/autodrive_devkit.zip" | sha256sum -c - && \
    python3 -m zipfile -e /tmp/autodrive_devkit.zip . && \
    rm -rf autodrive_devkit/test /tmp/autodrive_devkit.zip

# Copy in source code
COPY src/sim sim
COPY src/wato_test wato_test

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
# because 21.12 does not build on Python 3.12.
RUN pip3 install --no-cache-dir \
    "python-socketio==4.2.0" \
    "python-engineio==3.13.0" \
    "gevent==24.11.1" \
    "gevent-websocket==0.10.1"
