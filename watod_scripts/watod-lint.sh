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

# watod-lint.sh - Run .pre-commit-config.yaml hooks inside a lightweight Docker container.
# Hook virtualenvs are cached in a named Docker volume (watod_pre_commit_cache) so only
# the first run downloads hooks; subsequent runs take ~1-3 seconds.

MONO_DIR="$(cd -- "$(dirname -- "$(realpath "$0")")/.." && pwd)"

[[ -f "$MONO_DIR/watod-config.sh"       ]] && source "$MONO_DIR/watod-config.sh"
[[ -f "$MONO_DIR/watod-config.local.sh" ]] && source "$MONO_DIR/watod-config.local.sh"

source "$MONO_DIR/watod_scripts/watod-image-tags.sh"

export BUILDX_NO_DEFAULT_ATTESTATIONS="${BUILDX_NO_DEFAULT_ATTESTATIONS:-1}"

usage() {
  cat <<EOF
Usage: watod lint [OPTIONS] [FILES...] [-- PRE_COMMIT_ARGS...]

Runs pre-commit hooks from .pre-commit-config.yaml inside a lightweight Docker
container (cached in the 'watod_pre_commit_cache' volume).

Defaults to linting modified/untracked files in the working tree (or files changed
relative to main if the working tree is clean).

Options:
  -a, --all            Run hooks on all tracked files (--all-files)
  -b, --base [REF]     Lint all files changed relative to REF (default: main)
      --hook HOOK_ID   Run only the specified pre-commit hook ID
      --rebuild        Force rebuild of the lint Docker image
  -h, --help           Show this help and exit

Examples:
  watod lint                              # lint modified files (or branch diff vs main)
  watod lint --base                       # lint all files changed on this branch vs main
  watod lint --all                        # lint the entire repository
  watod lint watod docker/lint.Dockerfile # lint specific files
  watod lint --hook shellcheck watod      # run a single hook
EOF
}

RUN_ALL=false
USE_BASE=false
BASE_REF_ARG=""
HOOK_ID=""
FORCE_REBUILD=false
declare -a FILES=()
declare -a EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    -a|--all)
      RUN_ALL=true
      shift
      ;;
    -b|--base)
      USE_BASE=true
      if [[ $# -gt 1 && "$2" != -* && ! -e "$MONO_DIR/$2" && ! -e "$2" ]]; then
        BASE_REF_ARG="$2"
        shift 2
      else
        shift
      fi
      ;;
    --hook)
      shift
      HOOK_ID="${1:-}"
      [[ -z "$HOOK_ID" ]] && { echo "Error: --hook requires a hook ID" >&2; exit 1; }
      shift
      ;;
    --rebuild)
      FORCE_REBUILD=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    -*)
      EXTRA_ARGS+=("$1")
      shift
      ;;
    *)
      # Normalize host paths to repo-relative paths inside /repo
      f="$1"
      if [[ "$f" == "$MONO_DIR"/* ]]; then
        f="${f#"$MONO_DIR"/}"
      fi
      FILES+=("$f")
      shift
      ;;
  esac
done

# Derive repo-scoped image name + content hash of docker/lint.Dockerfile
if [[ -z "${REGISTRY_URL:-}" ]]; then
  repo_slug="${GITHUB_REPOSITORY:-}"
  if [[ -z "$repo_slug" ]]; then
    repo_slug="$(git -C "$MONO_DIR" remote get-url origin 2>/dev/null | sed -E 's#^(https://[^/]+/|git@[^:]+:)##; s#\.git$##' || true)"
  fi
  repo_slug="${repo_slug:-watonomous/wato_f1tenth}"
  REGISTRY_URL="ghcr.io/$(echo "$repo_slug" | tr '[:upper:]' '[:lower:]')"
fi

LINT_DOCKERFILE="$MONO_DIR/docker/lint.Dockerfile"
LINT_HASH="$(watod_short_hash < "$LINT_DOCKERFILE")"
LINT_IMAGE="${REGISTRY_URL}/lint:lint_${LINT_HASH}"
CACHE_VOLUME="${LINT_CACHE_VOLUME:-watod_pre_commit_cache}"

# Build the lint image once (shared across all branches until docker/lint.Dockerfile changes)
if [[ "$FORCE_REBUILD" == "true" ]] || ! docker image inspect "$LINT_IMAGE" >/dev/null 2>&1; then
  echo "Building lightweight lint image (${LINT_IMAGE})..."
  docker build -f "$LINT_DOCKERFILE" -t "$LINT_IMAGE" "$MONO_DIR/docker"
fi

# Determine target file set when neither --all nor explicit files were provided
declare -a PRE_COMMIT_ARGS=("run")
if [[ -n "$HOOK_ID" ]]; then
  PRE_COMMIT_ARGS+=("$HOOK_ID")
fi

if [[ "$RUN_ALL" == "true" ]]; then
  PRE_COMMIT_ARGS+=("--all-files")
elif [[ ${#FILES[@]} -gt 0 ]]; then
  PRE_COMMIT_ARGS+=("--files" "${FILES[@]}")
else
  declare -a changed=()
  if [[ "$USE_BASE" == "true" ]]; then
    ref="${BASE_REF_ARG:-$(watod_find_base_ref "$MONO_DIR")}"
    if [[ -z "$ref" ]]; then
      echo "Error: could not find base ref (pass --base <ref>)." >&2
      exit 1
    fi
    mapfile -t changed < <(
      {
        git -C "$MONO_DIR" diff --name-only --diff-filter=ACMR "$ref"
        git -C "$MONO_DIR" ls-files --others --exclude-standard
      } | LC_ALL=C sort -u
    )
  else
    # Working tree changes first (staged + unstaged + untracked)
    mapfile -t changed < <(
      {
        git -C "$MONO_DIR" diff --name-only --diff-filter=ACMR HEAD
        git -C "$MONO_DIR" ls-files --others --exclude-standard
      } | LC_ALL=C sort -u
    )
    # Fall back to branch diff vs main if working tree is clean
    if [[ ${#changed[@]} -eq 0 ]]; then
      ref="$(watod_find_base_ref "$MONO_DIR")"
      if [[ -n "$ref" ]]; then
        mapfile -t changed < <(git -C "$MONO_DIR" diff --name-only --diff-filter=ACMR "${ref}...HEAD" | LC_ALL=C sort -u)
      fi
    fi
  fi

  if [[ ${#changed[@]} -eq 0 ]]; then
    echo "No modified files to lint. Use 'watod lint --all' or pass file paths explicitly."
    exit 0
  fi

  PRE_COMMIT_ARGS+=("--files" "${changed[@]}")
fi

if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  PRE_COMMIT_ARGS+=("${EXTRA_ARGS[@]}")
fi

# Run as host UID/GID so auto-fixes keep host file ownership; share hook cache in named volume
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e PRE_COMMIT_HOME=/cache/pre-commit \
  -v "${MONO_DIR}:/repo" \
  -v "${CACHE_VOLUME}:/cache/pre-commit" \
  -w /repo \
  "$LINT_IMAGE" "${PRE_COMMIT_ARGS[@]}"
