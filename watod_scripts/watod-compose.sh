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
set -euo pipefail

# watod-compose.sh - Handles all docker compose operations with preprocessing
# Usage: watod-compose.sh <command> --pre-profiles <profile1> <profile2> ... --all-profiles <profile1> <profile2> ... [--pre-compose-files <files...>] [--compose-files <file1> <file2>...] [compose-args...]

# Helper function to run docker compose with standard settings
run_docker_compose() {
  DOCKER_BUILDKIT=${DOCKER_BUILDKIT:-1} \
    COMPOSE_BAKE=${COMPOSE_BAKE:-true} \
    docker compose "$@"
}

# Push images to the registry. Pushes run one at a time (images share large base
# layers and ghcr.io intermittently returns 500s on concurrent uploads) and retry
# with backoff on transient registry errors.
push_docker_compose() {
  local attempt
  for attempt in 1 2 3; do
    if COMPOSE_PARALLEL_LIMIT=1 run_docker_compose "$@" push; then
      return 0
    fi
    if [[ $attempt -lt 3 ]]; then
      echo "Push failed (attempt $attempt/3), retrying in $((attempt * 30))s..." >&2
      sleep $((attempt * 30))
    fi
  done
  echo "Error: push failed after 3 attempts" >&2
  return 1
}

# Parse arguments
COMPOSE_CMD=()
declare -a PRE_PROFILES=()
declare -a ALL_PROFILES=()
declare -a CUSTOM_PRE_COMPOSE_FILES=()
declare -a CUSTOM_ALL_COMPOSE_FILES=()
declare -a EXTRA_COMPOSE_ARGS=()

# First argument is the compose command
if [[ $# -lt 1 ]]; then
  echo "Error: Missing compose command" >&2
  echo "Usage: watod-compose.sh <command> --pre-profiles <profiles...> --all-profiles <profiles...> [--compose-files <files...>]" >&2
  exit 1
fi

COMPOSE_CMD=("$1")
shift

# Parse profile and file arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --pre-profiles)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        PRE_PROFILES+=("$1")
        shift
      done
      ;;
    --all-profiles)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        ALL_PROFILES+=("$1")
        shift
      done
      ;;
    --pre-compose-files)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        CUSTOM_PRE_COMPOSE_FILES+=("-f" "$1")
        shift
      done
      ;;
    --compose-files)
      shift
      while [[ $# -gt 0 && "$1" != --* ]]; do
        CUSTOM_ALL_COMPOSE_FILES+=("-f" "$1")
        shift
      done
      ;;
    *)
      # Additional compose command arguments (e.g., -d for up, --images for config)
      EXTRA_COMPOSE_ARGS+=("$1")
      shift
      ;;
  esac
done

# Get monorepo directory
MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MONO_DIR"

# Standard compose file sets
declare -a WATCLOUD_COMPOSE_FILES=()
if [[ "${WATCLOUD_MODE:-false}" == "true" ]]; then
  WATCLOUD_COMPOSE_FILES=("-f" "modules/docker-compose.watcloud.yaml")
fi

# Platform overlays from .env: car -> devices + nvidia runtime; a PC -> gamepad access
# (Linux with /dev/input) and the GPU for the GPU modules when GPU=nvidia
declare -a CAR_COMPOSE_FILES=()
if [[ -f "modules/.env" ]] && grep -q '^PLATFORM=car$' "modules/.env"; then
  CAR_COMPOSE_FILES=("-f" "modules/docker-compose.car.yaml")
else
  if [[ -d /dev/input ]]; then
    CAR_COMPOSE_FILES+=("-f" "modules/docker-compose.pc.yaml")
  fi
  if [[ -f "modules/.env" ]] && grep -q '^GPU=nvidia$' "modules/.env"; then
    CAR_COMPOSE_FILES+=("-f" "modules/docker-compose.gpu.yaml")
  fi
fi

declare -a DEFAULT_PRE_COMPOSE_FILES=("-f" "modules/docker-compose.yaml" "${WATCLOUD_COMPOSE_FILES[@]}" "-f" "modules/docker-compose.dep.yaml")
declare -a DEFAULT_ALL_COMPOSE_FILES=("-f" "modules/docker-compose.yaml" "${WATCLOUD_COMPOSE_FILES[@]}" "-f" "modules/docker-compose.dep.yaml" "-f" "modules/docker-compose.dev.yaml" "${CAR_COMPOSE_FILES[@]}")

# Use custom compose files if provided, otherwise use defaults
if [[ ${#CUSTOM_PRE_COMPOSE_FILES[@]} -gt 0 ]]; then
  PRE_COMPOSE_FILES=("${CUSTOM_PRE_COMPOSE_FILES[@]}")
else
  PRE_COMPOSE_FILES=("${DEFAULT_PRE_COMPOSE_FILES[@]}")
fi

if [[ ${#CUSTOM_ALL_COMPOSE_FILES[@]} -gt 0 ]]; then
  ALL_COMPOSE_FILES=("${CUSTOM_ALL_COMPOSE_FILES[@]}")
else
  ALL_COMPOSE_FILES=("${DEFAULT_ALL_COMPOSE_FILES[@]}")
fi

# Source modules/.env when present so base image checks and staleness warnings know active tags
if [[ -f "$MONO_DIR/modules/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$MONO_DIR/modules/.env"
  set +a
fi

# If build, pull base images for active modules only when not already present locally
if [[ "${COMPOSE_CMD[0]}" == "build" ]]; then
  declare -A active_bases=()
  arch="$(uname -m)"
  for p in "${PRE_PROFILES[@]}"; do
    module_name="${p%_pre}"
    dockerfile="docker/${module_name}.Dockerfile"
    [[ -f "$dockerfile" ]] || continue
    if grep -q '^ARG GPU' "$dockerfile" && [[ "${GPU:-none}" == "nvidia" ]]; then
      if [[ "$arch" == "aarch64" || "$arch" == "arm64" ]]; then
        [[ -n "${BASE_IMAGE_ARM64_NVIDIA:-}" ]] && active_bases["$BASE_IMAGE_ARM64_NVIDIA"]=1
      elif grep -q 'BASE_IMAGE_AMD64_NVIDIA' "$dockerfile"; then
        [[ -n "${BASE_IMAGE_AMD64_NVIDIA:-}" ]] && active_bases["$BASE_IMAGE_AMD64_NVIDIA"]=1
      else
        [[ -n "${BASE_IMAGE_CPU:-}" ]] && active_bases["$BASE_IMAGE_CPU"]=1
      fi
    else
      [[ -n "${BASE_IMAGE_CPU:-}" ]] && active_bases["$BASE_IMAGE_CPU"]=1
    fi
  done

  if [[ ${#active_bases[@]} -gt 0 ]]; then
    for base_image in "${!active_bases[@]}"; do
      if docker image inspect "$base_image" >/dev/null 2>&1; then
        continue
      fi
      echo "Pulling missing base image $base_image..."
      if ! docker pull "$base_image" 2>/dev/null; then
        echo "  Base image not found locally or in registry: $base_image" >&2
        echo "  Run './watod build-base' to build it locally." >&2
      fi
    done
  fi
fi

# Convert profile names to --profile flags
declare -a PRE_PROFILE_FLAGS=()
for p in "${PRE_PROFILES[@]}"; do
  PRE_PROFILE_FLAGS+=("--profile" "$p")
done

declare -a ALL_PROFILE_FLAGS=()
for p in "${ALL_PROFILES[@]}"; do
  ALL_PROFILE_FLAGS+=("--profile" "$p")
done

# If build, run PRE-BUILD stage (source and dependency stages)
if [[ "${COMPOSE_CMD[0]}" == "build" && ${#PRE_PROFILES[@]} -gt 0 ]]; then
  echo "RUNNING PRE-BUILD"
  run_docker_compose "${PRE_COMPOSE_FILES[@]}" "${PRE_PROFILE_FLAGS[@]}" build "${EXTRA_COMPOSE_ARGS[@]}"

  # In CI, push PRE-BUILD images to registry
  if [[ -n ${CI:-} || -n ${GITHUB_ACTIONS:-} ]]; then
    echo "CI detected: Pushing PRE-BUILD images to registry..."
    push_docker_compose "${PRE_COMPOSE_FILES[@]}" "${PRE_PROFILE_FLAGS[@]}"
  fi
fi

# Handle 'up' command preprocessing: always run in detached mode unless -d/--detach already passed
SHOW_STATUS_PANEL=false
if [[ "${COMPOSE_CMD[0]}" == "up" ]]; then
  has_detach=false
  for arg in "${EXTRA_COMPOSE_ARGS[@]}"; do
    if [[ "$arg" == "-d" || "$arg" == "--detach" ]]; then
      has_detach=true
      break
    fi
  done
  if [[ "$has_detach" == "false" ]]; then
    EXTRA_COMPOSE_ARGS+=( -d )
    SHOW_STATUS_PANEL=true
  fi

  # Warn if any active deploy image (floating local tag) was built from a different branch or checkout
  if [[ -n "${GIT_BRANCH:-}" && -n "${TAG:-}" ]]; then
    declare -A checked_profiles=()
    for p in "${ALL_PROFILES[@]}"; do
      [[ "$p" == *_dev || "$p" == "claude_code" ]] && continue
      [[ -n "${checked_profiles[$p]:-}" ]] && continue
      checked_profiles["$p"]=1
      mod_upper="$(echo "$p" | tr '[:lower:]' '[:upper:]')"
      img_var="${mod_upper}_IMAGE"
      img_repo="${!img_var:-}"
      [[ -z "$img_repo" ]] && continue
      img_ref="${img_repo}:${TAG}"
      if docker image inspect "$img_ref" >/dev/null 2>&1; then
        built_branch="$(docker image inspect --format '{{index .Config.Labels "org.watonomous.git_branch"}}' "$img_ref" 2>/dev/null || true)"
        built_commit="$(docker image inspect --format '{{index .Config.Labels "org.watonomous.git_commit"}}' "$img_ref" 2>/dev/null || true)"
        built_dir="$(docker image inspect --format '{{index .Config.Labels "org.watonomous.mono_dir"}}' "$img_ref" 2>/dev/null || true)"
        if [[ -n "$built_branch" && "$built_branch" != "unknown" && "$built_branch" != "$GIT_BRANCH" ]]; then
          echo -e "\033[0;33mWarning:\033[0m ${p}_bringup (${TAG}) was built from branch '${built_branch}' (${built_commit}), current branch is '${GIT_BRANCH}' (${GIT_COMMIT:-?})." >&2
          echo -e "         Run './watod -m ${p} build' if ${p} code changed on this branch." >&2
        elif [[ -n "$built_dir" && "$built_dir" != "unknown" && "$built_dir" != "$MONO_DIR" ]]; then
          echo -e "\033[0;33mWarning:\033[0m ${p}_bringup (${TAG}) was built from checkout '${built_dir}', current checkout is '${MONO_DIR}'." >&2
          echo -e "         Run './watod -m ${p} build' or set COMPOSE_PROJECT_NAME/TAG in watod-config.local.sh." >&2
        fi
      fi
    done
  fi
fi

# Run main compose command
if [[ "${COMPOSE_CMD[0]}" == "build" ]]; then
  echo "RUNNING BUILD"
fi

# Run compose command with or without profiles
run_docker_compose "${ALL_COMPOSE_FILES[@]}" "${ALL_PROFILE_FLAGS[@]}" "${COMPOSE_CMD[@]}" "${EXTRA_COMPOSE_ARGS[@]}"

# In CI, push final images after successful build
if [[ "${COMPOSE_CMD[0]}" == "build" && ( -n ${CI:-} || -n ${GITHUB_ACTIONS:-} ) && ${#ALL_PROFILES[@]} -gt 0 ]]; then
  echo "CI detected: Pushing final images to registry..."
  push_docker_compose "${ALL_COMPOSE_FILES[@]}" "${ALL_PROFILE_FLAGS[@]}"
fi

# Display status panel after 'up' command
if [[ "$SHOW_STATUS_PANEL" == "true" ]]; then
  # ANSI color codes
  readonly BLUE='\033[0;34m'
  readonly GREEN='\033[0;32m'
  readonly CYAN='\033[0;36m'
  readonly RED='\033[0;31m'
  readonly YELLOW='\033[0;33m'
  readonly RESET='\033[0m'
  readonly BOLD='\033[1m'

  # Source .env to get ports and project name
  if [[ -f "$MONO_DIR/modules/.env" ]]; then
    # shellcheck disable=SC1091
    set -a
    source "$MONO_DIR/modules/.env"
    set +a
  fi

  # Determine URLs based on connection type
  foxglove_url="ws://localhost:${FOXGLOVE_BRIDGE_PORT}"

  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    # SSH_CONNECTION format: client_ip client_port server_ip server_port
    server_ip=$(echo "$SSH_CONNECTION" | awk '{print $3}')
    foxglove_url="ws://${server_ip}:${FOXGLOVE_BRIDGE_PORT}"

    # Show SSH warning
    echo ""
    echo -e "${YELLOW}⚠️  SSH Connection Detected!${RESET}"
    echo ""
    echo "If the URLs below don't work, forward ports on your local machine:"
    ssh_ports="-L ${FOXGLOVE_BRIDGE_PORT}:localhost:${FOXGLOVE_BRIDGE_PORT}"
    echo -e "  ${CYAN}ssh ${ssh_ports} <host>${RESET}"
    echo ""
  fi

  # Get list of running containers in this project
  mapfile -t containers < <(docker ps --format "{{.Names}}" | grep -E "^${COMPOSE_PROJECT_NAME}-" | sort)

  # Display status panel
  echo ""
  echo -e "${BLUE}╔═══════════════════════════════════════════════════════════╗${RESET}"
  echo -e "${BLUE}║${RESET}  ${BOLD}WATonomous Services Started${RESET}                              ${BLUE}║${RESET}"
  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  echo -e "${BLUE}║${RESET}  ${GREEN}Running Containers:${RESET}                                      ${BLUE}║${RESET}"

  for container in "${containers[@]}"; do
    # Remove project prefix
    service_name="${container#"${COMPOSE_PROJECT_NAME}"-}"
    service_name="${service_name%-[0-9]*}"  # Remove trailing -1, -2, etc.

    # Check if it's a dev container
    if [[ "$service_name" == *"_dev" ]]; then
      # Format: "    - service_name (DevContainer)" padded to 59 chars
      printf "${BLUE}║${RESET}    - ${CYAN}%-37s${RESET} ${RED}(DevContainer)${RESET} ${BLUE}║${RESET}\n" "${service_name}"
    else
      # Format: "    - service_name" padded to 59 chars
      printf "${BLUE}║${RESET}    - ${CYAN}%-52s${RESET} ${BLUE}║${RESET}\n" "${service_name}"
    fi
  done

  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  printf "${BLUE}║${RESET}  ${GREEN}Foxglove:${RESET}    ${CYAN}%-43s${RESET} ${BLUE}║${RESET}\n" "${foxglove_url}"
  echo -e "${BLUE}╠═══════════════════════════════════════════════════════════╣${RESET}"
  echo -e "${BLUE}║${RESET}  View logs with ${BOLD}./watod logs -f <service>${RESET}.                  ${BLUE}║${RESET}"
  echo -e "${BLUE}║${RESET}  Use ${BOLD}./watod down${RESET} to stop all services.                   ${BLUE}║${RESET}"
  echo -e "${BLUE}╚═══════════════════════════════════════════════════════════╝${RESET}"
  echo ""
fi
