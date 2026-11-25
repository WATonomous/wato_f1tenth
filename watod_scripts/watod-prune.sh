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

# watod-prune.sh - Safely clean up unused Docker images belonging to this repository.
# Keeps:
#   - Any image currently referenced by a container (running or stopped)
#   - Base images and module tags currently configured in modules/.env
#   - Canonical shared base images (without branch/dirty suffixes)
# Never touches images belonging to other repositories.

MONO_DIR="$(cd -- "$(dirname -- "$(realpath "$0")")/.." && pwd)"

DRY_RUN=false
PRUNE_CACHE=false
PRUNE_BASES=false
ASSUME_YES=false

usage() {
  cat <<EOF
Usage: watod prune [OPTIONS]

Removes old/unused Docker images belonging to this repository (old branch tags,
superseded content-hashed deps/dev tags, and dangling intermediate images).
Images used by existing containers or the active modules/.env are preserved.

Options:
  -n, --dry-run   Show which images would be removed without deleting them
  -y, --yes       Skip confirmation prompt
  --bases         Also prune non-active branch-suffixed base images
  --cache         Also run 'docker builder prune' (affects all repos on host)
  -h, --help      Show this help and exit
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -n|--dry-run) DRY_RUN=true; shift ;;
    -y|--yes)     ASSUME_YES=true; shift ;;
    --bases)      PRUNE_BASES=true; shift ;;
    --cache)      PRUNE_CACHE=true; shift ;;
    -h|--help)    usage; exit 0 ;;
    *)            echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

# Source modules/.env if present so we protect the currently active image tags
if [[ -f "$MONO_DIR/modules/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$MONO_DIR/modules/.env"
  set +a
fi

# Determine repository prefix to scope cleanup strictly to this repo
if [[ -n "${REGISTRY_URL:-}" ]]; then
  REPO_PREFIX="${REGISTRY_URL}"
elif [[ -n "${INFRASTRUCTURE_IMAGE:-}" ]]; then
  REPO_PREFIX="${INFRASTRUCTURE_IMAGE%/infrastructure/infrastructure}"
else
  repo_slug=$(git -C "$MONO_DIR" remote get-url origin 2>/dev/null | sed -E 's#^(https://[^/]+/|git@[^:]+:)##; s#\.git$##' || true)
  repo_slug=${repo_slug:-watonomous/wato_f1tenth}
  REPO_PREFIX="ghcr.io/$(echo "$repo_slug" | tr '[:upper:]' '[:lower:]')"
fi

# Collect image refs & IDs to keep
declare -A KEEP_REFS=()
declare -A KEEP_IDS=()

add_keep_ref() {
  local ref="$1"
  [[ -z "$ref" ]] && return 0
  KEEP_REFS["$ref"]=1
  local img_id
  img_id="$(docker image inspect --format '{{.Id}}' "$ref" 2>/dev/null || true)"
  if [[ -n "$img_id" ]]; then
    KEEP_IDS["$img_id"]=1
  fi
}

# 1. Keep images used by any existing container (running or stopped)
while IFS= read -r cid_img; do
  if [[ -n "$cid_img" ]]; then
    KEEP_IDS["$cid_img"]=1
  fi
done < <(docker ps -a --format '{{.Image}}' 2>/dev/null | while IFS= read -r img; do
  docker image inspect --format '{{.Id}}' "$img" 2>/dev/null || true
done)

# 2. Keep active base images and canonical shared base images
for base_ref in \
  "${BASE_IMAGE_CPU:-}" \
  "${BASE_IMAGE_AMD64_NVIDIA:-}" \
  "${BASE_IMAGE_ARM64_NVIDIA:-}" \
  "${REPO_PREFIX}/base:jazzy-ubuntu24.04" \
  "${REPO_PREFIX}/base:cuda12.9-trt-jazzy-ubuntu24.04" \
  "${REPO_PREFIX}/base:jetson-cuda12.8-trt-jazzy-ubuntu24.04"; do
  add_keep_ref "$base_ref"
done

# 3. Keep active module tags from modules/.env
TAG_ACTIVE="${TAG:-local}"
for mod in infrastructure interfacing perception world_modeling action sim; do
  mod_upper="$(echo "$mod" | tr '[:lower:]' '[:upper:]')"
  img_var="${mod_upper}_IMAGE"
  deps_var="${mod_upper}_DEPS_TAG"
  dev_var="${mod_upper}_DEV_TAG"
  img="${!img_var:-${REPO_PREFIX}/${mod}/${mod}}"
  deps_tag="${!deps_var:-}"
  dev_tag="${!dev_var:-}"

  add_keep_ref "${img}:${TAG_ACTIVE}"
  add_keep_ref "${img}:source_${TAG_ACTIVE}"
  add_keep_ref "${img}:test_${TAG_ACTIVE}"
  if [[ -n "$deps_tag" ]]; then
    add_keep_ref "${img}:deps_${deps_tag}"
  fi
  if [[ -n "$dev_tag" ]]; then
    add_keep_ref "${img}:dev_${dev_tag}"
  fi
done

# Scan local images belonging to REPO_PREFIX
declare -a CANDIDATES=()
while IFS=$'\t' read -r repo tag img_id size; do
  [[ "$repo" == "${REPO_PREFIX}/"* ]] || continue
  ref="${repo}:${tag}"

  # Skip base images unless --bases was explicitly passed
  if [[ "$repo" == "${REPO_PREFIX}/base" && "$PRUNE_BASES" != "true" ]]; then
    continue
  fi

  # Skip explicitly protected refs
  if [[ -n "${KEEP_REFS[$ref]:-}" ]]; then
    continue
  fi

  # Never delete a tag pointing to an image ID in use by a container
  if [[ -n "${KEEP_IDS[$img_id]:-}" ]]; then
    # If another kept ref points to the same ID, removing only this stale tag is safe
    # as long as it is not the exact tag a container was created with; check exact tag match.
    in_use_by_container=false
    while IFS= read -r cimg; do
      if [[ "$cimg" == "$ref" ]]; then
        in_use_by_container=true
        break
      fi
    done < <(docker ps -a --format '{{.Image}}' 2>/dev/null)
    if [[ "$in_use_by_container" == "true" ]]; then
      continue
    fi
  fi

  CANDIDATES+=("${ref}"$'\t'"${size}")
done < <(docker images --no-trunc --format '{{.Repository}}\t{{.Tag}}\t{{.ID}}\t{{.Size}}' | grep -v '<none>' | sort -u)

if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "No unused ${REPO_PREFIX} image tags found."
else
  echo "Unused ${REPO_PREFIX} image tags (${#CANDIDATES[@]}):"
  for entry in "${CANDIDATES[@]}"; do
    IFS=$'\t' read -r ref size <<< "$entry"
    printf "  %-75s %s\n" "$ref" "$size"
  done

  if [[ "$DRY_RUN" == "true" ]]; then
    echo ""
    echo "(dry run — no images removed)"
  else
    if [[ "$ASSUME_YES" != "true" ]]; then
      echo ""
      read -r -p "Remove these ${#CANDIDATES[@]} tag(s) and dangling images? [y/N] " reply
      if [[ ! "$reply" =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
      fi
    fi

    for entry in "${CANDIDATES[@]}"; do
      IFS=$'\t' read -r ref _ <<< "$entry"
      docker rmi "$ref" || true
    done
    docker image prune -f >/dev/null
    echo "Pruned unused ${REPO_PREFIX} image tags and dangling images."
  fi
fi

if [[ "$PRUNE_CACHE" == "true" ]]; then
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "(dry run — would run: docker builder prune)"
  elif [[ "$ASSUME_YES" == "true" ]]; then
    docker builder prune -f
  else
    docker builder prune
  fi
fi
