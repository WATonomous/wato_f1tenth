#!/usr/bin/env bash
# Porto AutoDRIVE commands. Run one action per terminal; autonomy starts disabled.
set -eo pipefail
MPC_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$MPC_ROOT"
export ROS_DOMAIN_ID=0
MPC_ACTION=${1:-help}
if [ "$#" -gt 0 ]; then shift; fi

if [ "$MPC_ACTION" = api ]; then
  # This installed image speaks the vector-velocity/gzip-LiDAR API used by sim.exe.
  # The older official 2025 API expects V1 Speed and is incompatible with this binary.
  MPC_IMAGE=${AUTODRIVE_IMAGE:-sha256:86084593df851c02494745215a12b546f58d94dee4e4e075f884547216abf6fc}
  MPC_CONTAINER=wato-mpc-autodrive-api-current
  docker image inspect "$MPC_IMAGE" >/dev/null
  if docker container inspect "$MPC_CONTAINER" >/dev/null 2>&1; then
    if [ "$(docker inspect --format '{{.Image}}' "$MPC_CONTAINER")" != "$(docker image inspect --format '{{.Id}}' "$MPC_IMAGE")" ]; then
      echo "Existing $MPC_CONTAINER uses another image; resolve it before starting." >&2
      exit 1
    fi
    docker start "$MPC_CONTAINER"
  else
    docker run -d --name "$MPC_CONTAINER" --network host --ipc host -e ROS_DOMAIN_ID=0 \
      --entrypoint /bin/bash "$MPC_IMAGE" -lc \
      'source /opt/ros/humble/setup.bash; source /home/autodrive_devkit/install/setup.bash; exec ros2 run autodrive_roboracer autodrive_bridge'
  fi
  echo 'Connect the Porto Unity simulator to 127.0.0.1:4567; use Autonomous mode.'
  exit 0
fi

source /opt/ros/jazzy/setup.bash
if [ -f install-ros/setup.bash ]; then source install-ros/setup.bash; fi
# Host UDP avoids the Docker/WSL DDS shared-memory delivery failure observed here.
export FASTRTPS_DEFAULT_PROFILES_FILE="$MPC_ROOT/src/robot/mpc_controller/config/fastdds_udp.xml"
export ROS_LOG_DIR="$MPC_ROOT/log-autodrive/ros"
mkdir -p "$ROS_LOG_DIR"
case "$MPC_ACTION" in
  build)
    exec colcon --log-base log-ros build --build-base build-ros --install-base install-ros \
      --packages-up-to mpc_controller --cmake-args -DCMAKE_BUILD_TYPE=Release \
      "-DMPC_CONFIG=$MPC_ROOT/src/robot/mpc_controller/config/autodrive.json"
    ;;
  run) exec ros2 launch mpc_controller autodrive.launch.py "$@" ;;
  drive)
    echo 'Ctrl-C stops the enable heartbeat. Do not run another enable publisher.'
    exec ros2 topic pub -r 25 /dead_man_switch std_msgs/msg/Bool '{data: true}'
    ;;
  trial) exec ros2 run mpc_controller autodrive_trial.py "$@" ;;
  *)
    echo "Usage: bash documentation/run-mpc-autodrive.sh {build|api|run|drive|trial} [arguments]"
    echo 'Build once, start API, connect Unity, run stack, then drive OR a bounded trial.'
    ;;
esac
