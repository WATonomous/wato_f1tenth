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
# The reader runs inside the container, so a closed terminal or SSH session must not leave it
# publishing: this script stops it whenever it exits, including on hang-up (SIGHUP). A network drop
# the car's SSH server hasn't noticed yet is caught by the reader itself (its link check).
set -euo pipefail

if [[ "${TELEOP:-}" != "keyboard" ]]; then
  echo "Error: TELEOP=\"${TELEOP:-}\". Set TELEOP=\"keyboard\" in watod-config.local.sh, then ./watod up" >&2
  exit 1
fi
if [[ ! -t 0 ]]; then
  echo "Error: watod teleop reads keys, so it must run in a terminal" >&2
  exit 1
fi

# Capture the list first: `docker ps | grep -q` under pipefail can fail with SIGPIPE
running=$(docker ps --format '{{.Names}}')
container=""
for name in "${COMPOSE_PROJECT_NAME}-interfacing_bringup-1" "${COMPOSE_PROJECT_NAME}-interfacing_bringup_dev-1"; do
  if grep -qx "$name" <<< "$running"; then
    container="$name"
    break
  fi
done
if [[ -z "$container" ]]; then
  echo "Error: interfacing is not running. Start the stack first: ./watod up" >&2
  exit 1
fi

# One reader at a time (two would fight over /joy). The reader holds an flock on $lock for as long as
# it runs, so taking the lock is atomic and frees itself when the reader dies. $pidfile records the
# reader's PID with this session's token, so stop_reader only ever stops the reader it started.
lock=/tmp/keyboard_joy_node.lock
pidfile=/tmp/keyboard_joy_node.pid
token="$$-$RANDOM-$(date +%s%N)"
refused=3

# shellcheck disable=SC2317  # called from the EXIT trap
stop_reader() {
  docker exec "$container" bash -c "
    read -r pid tok < $pidfile 2>/dev/null || exit 0
    [ \"\$tok\" = \"$token\" ] || exit 0
    rm -f $pidfile
    kill -INT -- -\$pid 2>/dev/null || exit 0
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 \$pid 2>/dev/null || exit 0; sleep 0.2; done
    kill -KILL -- -\$pid 2>/dev/null" >/dev/null 2>&1 || true
}
trap stop_reader EXIT
trap 'exit 129' HUP
trap 'exit 143' TERM

# $$ is the container shell, which exec turns into the node itself (not `ros2 run`, whose Python
# wrapper ignores SIGINT and leaves the node running). docker exec -it makes it a process group leader.
# The node inherits fd 9, so it keeps the lock until it exits.
set +e
docker exec -it -e TELEOP_TOKEN="$token" "$container" bash -c '
  exec 9> '"$lock"'
  flock -n 9 || exit '"$refused"'
  echo "$$ $TELEOP_TOKEN" > '"$pidfile"'
  source /opt/ros/$ROS_DISTRO/setup.bash
  for f in /opt/watonomous/setup.bash /ws/install/setup.bash; do [ -f "$f" ] && source "$f"; done
  prefix=$(ros2 pkg prefix teleop_utils)
  config=$prefix/share/teleop_utils/config
  # allow_autonomy per platform; no PLATFORM falls back to the car (the stricter one)
  platform_config=$(ros2 pkg prefix interfacing_bringup)/share/interfacing_bringup/config/${PLATFORM:-car}/keyboard_teleop.yaml
  exec "$prefix/lib/teleop_utils/keyboard_joy_node" --ros-args \
    --params-file "$config/joypad_config.yaml" --params-file "$config/keyboard_config.yaml" \
    --params-file "$platform_config"
'
status=$?
set -e
if [[ $status -eq $refused ]]; then
  echo "Error: the keyboard reader is already running in $container (another terminal or SSH session)" >&2
fi
exit $status
