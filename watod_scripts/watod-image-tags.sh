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

# Shared image-tagging helpers sourced by watod-setup-env.sh and watod-build-base.sh.
#
# Design:
#   1. Base images (docker/base/*): share canonical tags across branches unless
#      docker/base/* differs from the default branch (main), in which case a
#      branch suffix is added automatically (or overridden via BASE_TAG_MODE /
#      BASE_TAG_SUFFIX).
#   2. Module deps & dev images: content-addressed per module (10-char SHA-256
#      over Dockerfile, template.Dockerfile, docker/config, and package.xmls) so
#      branches with identical dependencies reuse the same images automatically.
#   3. Module deploy/test/source images: floating "local" tag locally (or
#      "local-<COMPOSE_PROJECT_NAME>" when COMPOSE_PROJECT_NAME is customized on
#      a shared host), and branch tag in CI or when IMAGE_TAG_MODE=branch.

# Sanitize an arbitrary string for use as a Docker image tag (max 128 chars).
watod_sanitize_tag() {
  echo "$1" | sed -E 's/[^a-zA-Z0-9._-]/-/g; s/^-+//; s/-+$//' | cut -c1-128
}

# Compute a short deterministic SHA-256 hash from stdin.
watod_short_hash() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum | awk '{print substr($1, 1, 10)}'
  else
    shasum -a 256 | awk '{print substr($1, 1, 10)}'
  fi
}

# Find the reference branch to diff docker/base/* against (default: main or origin/main).
watod_find_base_ref() {
  local mono_dir="$1"
  if [[ -n "${BASE_REF:-}" ]]; then
    if git -C "$mono_dir" rev-parse --verify --quiet "$BASE_REF" >/dev/null 2>&1; then
      echo "$BASE_REF"
      return 0
    fi
  fi
  local candidate
  for candidate in main origin/main master origin/master; do
    if git -C "$mono_dir" rev-parse --verify --quiet "$candidate" >/dev/null 2>&1; then
      echo "$candidate"
      return 0
    fi
  done
  return 0
}

# Return 0 if any of the given repo-relative files differ from base_ref (including
# staged, unstaged, or untracked changes), 1 otherwise.
watod_files_differ_from_ref() {
  local mono_dir="$1"
  local base_ref="$2"
  shift 2
  local files=("$@")

  [[ -z "$base_ref" ]] && return 1
  command -v git >/dev/null 2>&1 || return 1

  # Compare base_ref against working tree (covers committed + staged + unstaged diffs)
  if ! git -C "$mono_dir" diff --quiet "$base_ref" -- "${files[@]}" 2>/dev/null; then
    return 0
  fi

  # Also detect untracked files matching the paths
  if [[ -n "$(git -C "$mono_dir" ls-files --others --exclude-standard -- "${files[@]}" 2>/dev/null)" ]]; then
    return 0
  fi

  return 1
}

# Resolve a base image tag for a specific base variant.
# Args: mono_dir canonical_tag branch base_tag_mode base_tag_suffix files...
watod_resolve_base_tag() {
  local mono_dir="$1"
  local canonical_tag="$2"
  local branch="$3"
  local mode="$4"
  local explicit_suffix="$5"
  shift 5
  local files=("$@")

  # Explicit suffix always wins
  if [[ -n "$explicit_suffix" ]]; then
    local clean_suffix
    clean_suffix="$(watod_sanitize_tag "${explicit_suffix#-}")"
    echo "${canonical_tag}-${clean_suffix}"
    return 0
  fi

  local sanitized_branch
  sanitized_branch="$(watod_sanitize_tag "$branch")"

  case "$mode" in
    shared)
      echo "$canonical_tag"
      ;;
    branch)
      echo "${canonical_tag}-${sanitized_branch}"
      ;;
    auto|*)
      local base_ref
      base_ref="$(watod_find_base_ref "$mono_dir")"
      if watod_files_differ_from_ref "$mono_dir" "$base_ref" "${files[@]}"; then
        if [[ "$branch" == "main" || "$branch" == "master" ]]; then
          local dirty_hash
          dirty_hash="$(
            for f in "${files[@]}"; do
              [[ -f "$mono_dir/$f" ]] && { echo "FILE:$f"; cat "$mono_dir/$f"; }
            done | watod_short_hash
          )"
          echo "${canonical_tag}-dirty-${dirty_hash}"
        else
          echo "${canonical_tag}-${sanitized_branch}"
        fi
      else
        echo "$canonical_tag"
      fi
      ;;
  esac
}

# Resolve all base image variables (BASE_REPO, BASE_TAG_*, BASE_IMAGE_*) and export them.
# Expects: MONO_DIR, BRANCH (optional), IMAGE_TAG_MODE (optional), BASE_TAG_MODE (optional),
#          BASE_TAG_SUFFIX (optional), BASE_REPO (optional), ROS_DISTRO (optional).
watod_resolve_base_images() {
  local mono_dir="${MONO_DIR}"
  local branch="${BRANCH:-}"
  if [[ -z "$branch" ]] && command -v git >/dev/null 2>&1; then
    branch="$(git -C "$mono_dir" branch --show-current 2>/dev/null || true)"
  fi
  branch="${branch:-main}"

  ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  if [[ -z "${BASE_REPO:-}" ]]; then
    if [[ -n "${REGISTRY_URL:-}" ]]; then
      BASE_REPO="${REGISTRY_URL}/base"
    else
      local repo_slug="${GITHUB_REPOSITORY:-}"
      if [[ -z "$repo_slug" ]] && command -v git >/dev/null 2>&1; then
        repo_slug="$(git -C "$mono_dir" remote get-url origin 2>/dev/null | sed -E 's#^(https://[^/]+/|git@[^:]+:)##; s#\.git$##' || true)"
      fi
      repo_slug="${repo_slug:-watonomous/wato_f1tenth}"
      BASE_REPO="ghcr.io/$(echo "$repo_slug" | tr '[:upper:]' '[:lower:]')/base"
    fi
  fi

  local effective_mode="${BASE_TAG_MODE:-}"
  if [[ -z "$effective_mode" ]]; then
    if [[ "${IS_CI:-false}" == "true" ]]; then
      effective_mode="auto"
    else
      case "${IMAGE_TAG_MODE:-auto}" in
        branch) effective_mode="branch" ;;
        shared) effective_mode="shared" ;;
        *)      effective_mode="auto" ;;
      esac
    fi
  fi
  BASE_TAG_MODE="$effective_mode"

  local suffix="${BASE_TAG_SUFFIX:-}"

  local cpu_files=(
    "docker/base/inject_ros2.Dockerfile"
    "docker/base/inject_wato_base.Dockerfile"
  )
  local gpu_files=(
    "${cpu_files[@]}"
    "docker/base/inject_tensorrt.Dockerfile"
  )
  local jetson_files=(
    "${cpu_files[@]}"
    "docker/base/inject_jetson_cuda.Dockerfile"
  )

  BASE_TAG_CPU="$(watod_resolve_base_tag "$mono_dir" "${ROS_DISTRO}-ubuntu24.04" "$branch" "$BASE_TAG_MODE" "$suffix" "${cpu_files[@]}")"
  BASE_TAG_AMD64_NVIDIA="$(watod_resolve_base_tag "$mono_dir" "cuda12.9-trt-${ROS_DISTRO}-ubuntu24.04" "$branch" "$BASE_TAG_MODE" "$suffix" "${gpu_files[@]}")"
  BASE_TAG_ARM64_NVIDIA="$(watod_resolve_base_tag "$mono_dir" "jetson-cuda12.8-trt-${ROS_DISTRO}-ubuntu24.04" "$branch" "$BASE_TAG_MODE" "$suffix" "${jetson_files[@]}")"

  BASE_IMAGE_CPU="${BASE_REPO}:${BASE_TAG_CPU}"
  BASE_IMAGE_AMD64_NVIDIA="${BASE_REPO}:${BASE_TAG_AMD64_NVIDIA}"
  BASE_IMAGE_ARM64_NVIDIA="${BASE_REPO}:${BASE_TAG_ARM64_NVIDIA}"

  export ROS_DISTRO BASE_REPO BASE_TAG_MODE
  export BASE_TAG_CPU BASE_TAG_AMD64_NVIDIA BASE_TAG_ARM64_NVIDIA
  export BASE_IMAGE_CPU BASE_IMAGE_AMD64_NVIDIA BASE_IMAGE_ARM64_NVIDIA
}

# Compute deterministic 10-char hash for a module's dependencies stage (docker/<mod>.Dockerfile).
# Args: mono_dir module gpu
watod_module_deps_hash() {
  local mono_dir="$1"
  local module="$2"
  local gpu="${3:-none}"
  local dockerfile="$mono_dir/docker/${module}.Dockerfile"

  {
    echo "MODULE=${module}"
    echo "BASE_IMAGE_CPU=${BASE_IMAGE_CPU}"
    if [[ -f "$dockerfile" ]] && grep -q '^ARG GPU' "$dockerfile"; then
      echo "GPU=${gpu}"
      echo "BASE_IMAGE_AMD64_NVIDIA=${BASE_IMAGE_AMD64_NVIDIA}"
      echo "BASE_IMAGE_ARM64_NVIDIA=${BASE_IMAGE_ARM64_NVIDIA}"
    fi
    if [[ -f "$dockerfile" ]]; then
      echo "FILE:docker/${module}.Dockerfile"
      cat "$dockerfile"
    fi
  } | watod_short_hash
}

# Compute deterministic 10-char hash for a module's develop stage (rosdep + dev tools + user).
# Automatically discovers package.xml files from every `COPY src/...` path in docker/<mod>.Dockerfile.
# Args: mono_dir module deps_hash claude_code username setuid setgid
watod_module_dev_hash() {
  local mono_dir="$1"
  local module="$2"
  local deps_hash="$3"
  local claude_code="${4:-true}"
  local username="${5:-root}"
  local setuid="${6:-0}"
  local setgid="${7:-0}"
  local dockerfile="$mono_dir/docker/${module}.Dockerfile"

  {
    echo "DEPS_HASH=${deps_hash}"
    echo "CLAUDE_CODE=${claude_code}"
    echo "USER=${username}:${setuid}:${setgid}"

    if [[ -f "$mono_dir/docker/template.Dockerfile" ]]; then
      echo "FILE:docker/template.Dockerfile"
      cat "$mono_dir/docker/template.Dockerfile"
    fi

    # Config files copied into the image
    if [[ -d "$mono_dir/docker/config" ]]; then
      local cfg
      while IFS= read -r cfg; do
        [[ -f "$cfg" ]] || continue
        echo "FILE:${cfg#"$mono_dir"/}"
        cat "$cfg"
      done < <(find "$mono_dir/docker/config" -maxdepth 1 -type f | LC_ALL=C sort)
    fi

    # Discover all src/ paths copied by this module's Dockerfile and hash their package.xml files
    if [[ -f "$dockerfile" ]]; then
      local -a src_dirs=()
      local src_rel
      while IFS= read -r src_rel; do
        if [[ -n "$src_rel" && -d "$mono_dir/$src_rel" ]]; then
          src_dirs+=("$mono_dir/$src_rel")
        fi
      done < <(awk '$1 == "COPY" && $2 ~ /^src\// {print $2}' "$dockerfile" | LC_ALL=C sort -u)

      if [[ ${#src_dirs[@]} -gt 0 ]]; then
        local pkg_xml
        while IFS= read -r pkg_xml; do
          [[ -f "$pkg_xml" ]] || continue
          echo "FILE:${pkg_xml#"$mono_dir"/}"
          cat "$pkg_xml"
        done < <(find "${src_dirs[@]}" -name package.xml -type f | LC_ALL=C sort -u)
      fi
    fi
  } | watod_short_hash
}
