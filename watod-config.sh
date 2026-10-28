#!/bin/bash
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

## ----------------------- watod Configuration File Override ----------------------------

##
## HINT: You can copy the contents of this file to a watod-config.local.sh
##       file that is untrackable by git and readable by watod.
##

############################ ACTIVE MODULE CONFIGURATION ############################
## List of active modules to run, defined in docker-compose.yaml.
## Add ":dev" suffix to run a module in development mode (with live code editing)
##
## Possible values:
##   - interfacing          :   car drivers (VESC, Hokuyo), teleop, ackermann_mux
##   - perception           :   perception nodes (empty for now)
##   - world_modeling       :   localization, mapping, global planner
##   - action               :   control: CONTROLLER (pure pursuit, MPPI) and LOCAL_PLANNER
##   - sim                  :   AutoDRIVE sim adapter (simulator runs on the host)
##
## Examples:
##   - "interfacing world_modeling action sim"   : full stack in sim (PLATFORM=sim)
##   - "interfacing world_modeling action"       : full stack on the car (PLATFORM=car)
##   - "interfacing"                             : car drivers and teleop only (was jetson_up)
##   - "action:dev"                              : action in dev mode (editable)
##
## DEFAULT = the full stack for the detected platform:
##   car: "interfacing world_modeling action"; PC: "interfacing world_modeling action sim"

# export ACTIVE_MODULES=""

############################## ADVANCED CONFIGURATIONS ##############################
## Install Claude Code in dev containers. Set to "false" to skip installation.
## DEFAULT = "true"
# export CLAUDE_CODE="false"

## Enable WATCloud mode for rootless Docker environments.
## Adds a shared network namespace container so services can communicate
## without requiring network_mode: host.
## DEFAULT = "false"
# export WATCLOUD_MODE="false"

## Name to append to docker containers. DEFAULT = "<your_watcloud_username>"
# export COMPOSE_PROJECT_NAME=""

## Tag to use. Images are formatted as <IMAGE_NAME>:<TAG> with forward slashes replaced with dashes.
## DEFAULT = "<your_current_github_branch>"
# export TAG=""

# Docker Registry to pull/push images. DEFAULT = "ghcr.io/<owner>/<repo>" from the git origin remote
# export REGISTRY_URL=""

# Directory where bags are stored and read. DEFAULT = "$MONO_DIR/bags"
# export BAG_DIRECTORY=""

############################### ROS 2 MIDDLEWARE SETTINGS ##############################
## Middleware to use for interprocess communication. DEFAULT = "rmw_cyclonedds_cpp"
## (f1tenth does not use Zenoh; the Zenoh settings below are unused)
# export RMW_IMPLEMENTATION=""

## Zenoh configuration - enables zero-copy transport for local communication
## Path to Zenoh router configuration. DEFAULT = "file:///opt/watonomous/rmw_zenoh_router_config.json5"
# export ZENOH_ROUTER_CONFIG_URI=""

## Path to Zenoh session configuration. DEFAULT ="file:///opt/watonomous/rmw_zenoh_session_config.json5"
# export ZENOH_SESSION_CONFIG_URI=""


## ROS 2 Domain ID. DEFAULT = "0", shared by laptops and the car so they see each other.
## Set a different value to isolate yourself on a busy network.
# export ROS_DOMAIN_ID=""

############################### F1TENTH PLATFORM ##################################
## car or sim. DEFAULT = detected: "car" on a Jetson (/etc/nv_tegra_release), else "sim"
# export PLATFORM="car"

## Run slam_toolbox to build a map instead of localizing. DEFAULT = "false"
# export MAPPING="true"

## Controller in the action module: a file name in action_bringup/launch/controllers/
## (pure_pursuit, mppi, none). DEFAULT = "pure_pursuit"
# export CONTROLLER="pure_pursuit"

## Local planner in the action module: a file name in action_bringup/launch/local_planners/
## (none). DEFAULT = "none"
# export LOCAL_PLANNER="none"

## Track: a folder in world_modeling/race_tracks/tracks/ (wato_office, e7_fifth_v3, e7_fifth_big,
## autodrive). Picks the localization map, the global planner's raceline and MPPI's.
## DEFAULT = "wato_office" on the car, "autodrive" in sim
# export TRACK="wato_office"

## Start the Intel RealSense driver in interfacing (IMU on /camera/camera/imu). DEFAULT = "false"
# export REALSENSE="true"

## Also stream RealSense color + depth (640x480 @ 30 Hz), recorded as JPEG / zstd by
## `watod bag record --preset car`. DEFAULT = "false"
# export REALSENSE_IMAGES="true"

## Topic preset for `watod bag record` with no topics given (infrastructure_bringup/config/
## bag_presets/<name>.txt). DEFAULT = the platform: "car" or "sim". "" = no default.
# export BAG_PRESET="car"

## CPUs the bag recorder container may use (docker --cpuset-cpus), so recording can't
## starve the particle filter. DEFAULT = "4,5" on the car, no limit elsewhere ("" = no limit)
# export BAG_CPUSET="4,5"

## Source of /hal/imu: realsense, vesc (not working as intended yet), none or auto.
## DEFAULT = "auto" (car: realsense when REALSENSE=true; sim: AutoDRIVE's IMU)
# export IMU="realsense"

## What publishes /hal/localization/odom: pf (particle filter), truth (sim ground truth) or none.
## DEFAULT = "auto" (car: pf; sim: truth)
# export LOCALIZATION="pf"

## Publish the particle filter's /pf/viz/* debug topics (particles, fake scan, pose). They cost
## it extra work per update, so leave them off unless you are debugging localization.
## DEFAULT = "false"
# export PF_VIZ="true"

## GPU images: nvidia or none. DEFAULT = detected (Jetson or a working nvidia-smi + Docker
## nvidia runtime -> nvidia, else none). none builds action/perception/world_modeling on the CPU base.
# export GPU="none"

############################### FOXGLOVE SETTINGS ##################################
## Size of the outgoing websocket buffer (bytes) for foxglove_bridge. Increase to prevent drops; default is 256 MiB.
# export FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES="${FOXGLOVE_BRIDGE_SEND_BUFFER_LIMIT_BYTES:-268435456}"
