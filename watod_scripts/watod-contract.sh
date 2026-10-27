#!/usr/bin/env bash
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

# Check the running stack against the /hal topic contract (hal_contract/config/contract.yaml).
# Runs hal_contract's checker in the infrastructure container with this machine's PLATFORM,
# IMU, LOCALIZATION, MAPPING and REALSENSE. Extra arguments go to the checker, e.g.
#   watod contract --group sensors --duration 10
set -euo pipefail

infra_container="${COMPOSE_PROJECT_NAME}-infrastructure_bringup-1"
if ! docker ps --format '{{.Names}}' | grep -q "^${infra_container}$"; then
  echo "Error: '${infra_container}' is not running. Start the stack first: ./watod up"
  exit 1
fi

docker exec "$infra_container" bash -c \
  'source /opt/ros/$ROS_DISTRO/setup.bash && source /opt/watonomous/setup.bash && ros2 run hal_contract check "$@"' \
  _ --platform "$PLATFORM" --imu "$IMU" --localization "$LOCALIZATION" \
  --mapping "$MAPPING" --realsense "$REALSENSE" "$@"
