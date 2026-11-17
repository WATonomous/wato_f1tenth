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

# Drive with the keyboard (TELEOP=keyboard): start teleop_utils' keyboard_joy_node in this terminal,
# inside the running interfacing container. It reads keys from the terminal and publishes /joy for
# joypad_node, which interfacing.launch.yaml already runs. Works over SSH on the car.
#
# The reader runs inside the container, so a dropped SSH session or a closed terminal must not leave
# it publishing: this script stops it whenever it exits, including on hang-up (SIGHUP).
set -euo pipefail

if [[ "${TELEOP:-}" != "keyboard" ]]; then
  echo "Error: TELEOP=\"${TELEOP:-}\". Set TELEOP=\"keyboard\" in watod-config.local.sh, then ./watod up" >&2
  exit 1
fi
if [[ ! -t 0 ]]; then
  echo "Error: watod teleop reads keys, so it must run in a terminal" >&2
  exit 1
fi

container=""
for name in "${COMPOSE_PROJECT_NAME}-interfacing_bringup-1" "${COMPOSE_PROJECT_NAME}-interfacing_bringup_dev-1"; do
  if docker ps --format '{{.Names}}' | grep -q "^${name}$"; then
    container="$name"
    break
  fi
done
if [[ -z "$container" ]]; then
  echo "Error: interfacing is not running. Start the stack first: ./watod up" >&2
  exit 1
fi

# Two readers would both publish /joy and fight over the buttons
pidfile=/tmp/keyboard_joy_node.pid
if docker exec "$container" sh -c "[ -f $pidfile ] && kill -0 \$(cat $pidfile) 2>/dev/null"; then
  echo "Error: the keyboard reader is already running in $container (another terminal or SSH session)" >&2
  exit 1
fi

# SIGINT to the reader's process group: it sends one released /joy and exits. If it is still there
# after 2 s, kill it; /joy then stops and joypad_node's watchdog releases the dead man.
stop_reader() {
  docker exec "$container" bash -c "
    [ -f $pidfile ] || exit 0
    pid=\$(cat $pidfile); rm -f $pidfile
    kill -INT -- -\$pid 2>/dev/null || exit 0
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 \$pid 2>/dev/null || exit 0; sleep 0.2; done
    kill -KILL -- -\$pid 2>/dev/null" >/dev/null 2>&1 || true
}
trap stop_reader EXIT
trap 'exit 129' HUP
trap 'exit 143' TERM

# $$ is the container shell, which exec turns into the node itself (not `ros2 run`, whose Python
# wrapper ignores SIGINT and leaves the node running). docker exec -it makes it a process group leader.
docker exec -it "$container" bash -c '
  echo $$ > '"$pidfile"'
  source /opt/ros/$ROS_DISTRO/setup.bash
  for f in /opt/watonomous/setup.bash /ws/install/setup.bash; do [ -f "$f" ] && source "$f"; done
  prefix=$(ros2 pkg prefix teleop_utils)
  config=$prefix/share/teleop_utils/config
  exec "$prefix/lib/teleop_utils/keyboard_joy_node" --ros-args \
    --params-file "$config/joypad_config.yaml" --params-file "$config/keyboard_config.yaml"
'
