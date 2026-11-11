#!/usr/bin/env bash
# Run one of the headless VESC Tool scripts in this folder, ON THE CAR (the Jetson), and log it.
#
#   agents/scripts/vesc/run_vesc_script.sh save_configs        # read-only backup of the motor + app config
#   agents/scripts/vesc/run_vesc_script.sh set_mcconf          # apply ~/vesc_configs/changes.json (see set_mcconf.qml)
#   agents/scripts/vesc/run_vesc_script.sh offground_test      # MOVES THE MOTOR: wheels off the ground
#
# Needs VESC Tool built on the car (agents/workflows/vesc-firmware.md) and the VESC free: the stack's
# vesc_driver holds the serial port, so stop the stack first (./watod down, ask before running it).
# Logs go to ~/vesc_configs/tests/, config snapshots to ~/vesc_configs/history/ and backups/.
set -euo pipefail

NAME=${1:?usage: run_vesc_script.sh <script name without .qml>}
HERE="$(cd -- "$(dirname -- "$0")" && pwd)"
SCRIPT="$HERE/$NAME.qml"
BASE="$HOME/vesc_configs"
TOOL="${VESC_TOOL:-$HOME/vesc_tool/build/lin/vesc_tool_6.00}"

[[ -f "$SCRIPT" ]] || { echo "no such script: $SCRIPT" >&2; exit 1; }
[[ -x "$TOOL" ]] || { echo "VESC Tool not found at $TOOL (build it: agents/workflows/vesc-firmware.md)" >&2; exit 1; }
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q 'interfacing_bringup'; then
  echo "the stack's interfacing container is running and holds the VESC: stop it first (./watod down)" >&2
  exit 1
fi
mkdir -p "$BASE/latest" "$BASE/history" "$BASE/tests" "$BASE/backups"

STAMP=$(date +%F_%H%M%S)
LOG="$BASE/tests/${STAMP}_${NAME}.log"
echo "running $NAME (log: $LOG)"
QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software QML_XHR_ALLOW_FILE_READ=1 \
  timeout 300 "$TOOL" --loadQml "$SCRIPT" 2>&1 | grep -E '^qml: ' | sed 's/^qml: //' | tee "$LOG"

if [[ "$NAME" == "save_configs" && -f "$BASE/latest/mcconf.xml" ]]; then
  mkdir -p "$BASE/backups/$STAMP"
  cp "$BASE/latest/"*.xml "$BASE/backups/$STAMP/"
  echo "backup: $BASE/backups/$STAMP"
fi
