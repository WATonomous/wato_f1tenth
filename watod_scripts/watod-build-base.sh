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

# Build the f1tenth base images locally, the same way the template's
# build_base_images workflow does (inject_ros2 -> inject_wato_base), plus the
# GPU layers. Nothing is pushed: images stay in the local Docker image store
# until the prebuilt-images step publishes them.
#
#   cpu     ubuntu:24.04 + ROS 2 Jazzy                          (amd64 and arm64)
#   gpu     CUDA 12.9 + cuDNN (devel) + TensorRT + ROS 2 Jazzy  (amd64 PCs with an NVIDIA GPU)
#   jetson  ubuntu:24.04 + CUDA 12.8 + cuDNN + TensorRT + Jazzy (arm64, build on the Jetson)

set -euo pipefail

MONO_DIR="$(cd -- "$(dirname -- "$(realpath "$0")")/.." && pwd)"
BASE_REPO=${BASE_REPO:-ghcr.io/watonomous/wato_f1tenth/base}
ROS_DISTRO=${ROS_DISTRO:-jazzy}

CPU_TAG="${ROS_DISTRO}-ubuntu24.04"
GPU_TAG="cuda12.9-trt-${ROS_DISTRO}-ubuntu24.04"
JETSON_TAG="jetson-cuda12.8-trt-${ROS_DISTRO}-ubuntu24.04"

usage() {
  cat <<EOF
Usage: watod build-base [cpu|gpu|jetson|all]

Builds f1tenth base images locally as ${BASE_REPO}:<tag>.
  cpu     ${CPU_TAG}
  gpu     ${GPU_TAG}      (x86_64 only)
  jetson  ${JETSON_TAG}   (aarch64 only)
  all     the CPU base, plus the GPU base if this machine has an NVIDIA GPU (default)
EOF
}

# inject ROS 2 into GENERIC, then the WATO base layer, tagged BASE_REPO:TAG
build_ros_base() {
  local generic=$1 tag=$2
  echo "==> ${BASE_REPO}:${tag} (from ${generic})"
  docker build -f "$MONO_DIR/docker/base/inject_ros2.Dockerfile" --target devel \
    --build-arg GENERIC_IMAGE="$generic" --build-arg ROS_DISTRO="$ROS_DISTRO" \
    -t "${BASE_REPO}:${tag}_generic" "$MONO_DIR"
  docker build -f "$MONO_DIR/docker/base/inject_wato_base.Dockerfile" --target wato_base \
    --build-arg GENERIC_IMAGE="${BASE_REPO}:${tag}_generic" \
    -t "${BASE_REPO}:${tag}" "$MONO_DIR"
}

build_cpu() {
  build_ros_base "ubuntu:24.04" "$CPU_TAG"
}

build_gpu() {
  [[ $(uname -m) == "x86_64" ]] || { echo "gpu base is x86_64 only"; exit 1; }
  docker build -f "$MONO_DIR/docker/base/inject_tensorrt.Dockerfile" --target tensorrt \
    --build-arg GENERIC_IMAGE="nvcr.io/nvidia/cuda:12.9.1-cudnn-devel-ubuntu24.04" \
    -t "${BASE_REPO}:${GPU_TAG}_cuda" "$MONO_DIR"
  build_ros_base "${BASE_REPO}:${GPU_TAG}_cuda" "$GPU_TAG"
}

build_jetson() {
  [[ $(uname -m) == "aarch64" ]] || { echo "jetson base is aarch64 only; build it on the Jetson"; exit 1; }
  docker build -f "$MONO_DIR/docker/base/inject_jetson_cuda.Dockerfile" --target jetson_cuda \
    --build-arg GENERIC_IMAGE="ubuntu:24.04" \
    -t "${BASE_REPO}:${JETSON_TAG}_cuda" "$MONO_DIR"
  build_ros_base "${BASE_REPO}:${JETSON_TAG}_cuda" "$JETSON_TAG"
}

case "${1:-all}" in
  cpu) build_cpu ;;
  gpu) build_gpu ;;
  jetson) build_jetson ;;
  all)
    build_cpu
    if [[ -f /etc/nv_tegra_release ]]; then
      build_jetson
    elif [[ $(uname -m) == "x86_64" ]] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
      build_gpu
    else
      echo "No NVIDIA GPU found: built the CPU base only (GPU=none uses it for every module)."
    fi
    ;;
  -h|--help|help) usage ;;
  *) usage; exit 1 ;;
esac
