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

# Clone or update the tool repos in tools/ (git-ignored here, not submodules), so each one follows
# its default branch instead of a pinned commit.
#
#   watod tools          clone the missing ones, pull the rest to the latest default branch
#   watod tools status   branch, commit and how far behind each one is
#
# A repo that is on another branch or has uncommitted changes is fetched but never pulled, so
# local work is never touched.

set -uo pipefail

MONO_DIR="$(cd -- "$(dirname -- "$(realpath "$0")")/.." && pwd)"
TOOLS_DIR="$MONO_DIR/tools"

# name  url (HTTPS, so cloning needs no SSH key; the repos are public)
REPOS=(
  "wato_f1tenth_raceline_generator https://github.com/WATonomous/wato_f1tenth_raceline_generator.git"
  "wato_f1tenth_model_fram https://github.com/WATonomous/wato_f1tenth_model_fram.git"
)

default_branch() {
  # the remote's default branch (main or master), from origin/HEAD
  local ref
  ref=$(git -C "$1" symbolic-ref --quiet --short refs/remotes/origin/HEAD 2>/dev/null) \
    || { git -C "$1" remote set-head origin --auto >/dev/null 2>&1; \
         ref=$(git -C "$1" symbolic-ref --quiet --short refs/remotes/origin/HEAD 2>/dev/null); }
  echo "${ref#origin/}"
}

sync_repo() {
  local name=$1 url=$2 dir="$TOOLS_DIR/$1"
  if [[ ! -d "$dir/.git" ]]; then
    echo "== $name: cloning"
    git clone --quiet "$url" "$dir" || { echo "   clone failed"; return 1; }
    echo "   $(git -C "$dir" branch --show-current) at $(git -C "$dir" log --oneline -1)"
    return 0
  fi
  echo "== $name"
  git -C "$dir" fetch --quiet origin || { echo "   fetch failed"; return 1; }
  local branch current
  branch=$(default_branch "$dir")
  current=$(git -C "$dir" branch --show-current)
  if [[ "$current" != "$branch" ]]; then
    echo "   on '$current', not '$branch': fetched only (switch with: git -C tools/$name switch $branch)"
  elif [[ -n "$(git -C "$dir" status --porcelain --untracked-files=no)" ]]; then
    echo "   uncommitted changes: fetched only"
  elif git -C "$dir" pull --quiet --ff-only origin "$branch"; then
    echo "   $branch at $(git -C "$dir" log --oneline -1)"
  else
    echo "   $branch has diverged from origin: fetched only, merge or rebase it yourself"
  fi
}

status_repo() {
  local name=$1 dir="$TOOLS_DIR/$1"
  if [[ ! -d "$dir/.git" ]]; then
    echo "== $name: not cloned (run: watod tools)"
    return
  fi
  local branch behind
  branch=$(default_branch "$dir")
  behind=$(git -C "$dir" rev-list --count "HEAD..origin/$branch" 2>/dev/null || echo "?")
  echo "== $name: on $(git -C "$dir" branch --show-current) at $(git -C "$dir" log --oneline -1)"
  echo "   $behind commit(s) behind origin/$branch (as of the last fetch)"
}

mkdir -p "$TOOLS_DIR"
case "${1:-sync}" in
  sync)
    rc=0
    for entry in "${REPOS[@]}"; do
      read -r name url <<< "$entry"
      sync_repo "$name" "$url" || rc=1
    done
    exit $rc
    ;;
  status)
    for entry in "${REPOS[@]}"; do
      read -r name _ <<< "$entry"
      status_repo "$name"
    done
    ;;
  *)
    echo "Usage: watod tools [sync|status]"
    exit 1
    ;;
esac
