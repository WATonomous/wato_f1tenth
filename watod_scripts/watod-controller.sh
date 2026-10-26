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

# Pair and connect a Bluetooth gamepad (PS4 DualShock 4 / PS5 DualSense) from the
# command line, on the host (the containers see the pad through /dev/input).
#
#   watod controller pair      scan, remove any stale pairing, pair + trust + connect
#   watod controller connect   reconnect the saved pad
#   watod controller status    adapter, pad connection and /dev/input/js* devices
#   watod controller forget    remove the pairing and the saved MAC
#
# The pad's MAC is saved as CONTROLLER_MAC in watod-config.local.sh.

set -euo pipefail

MONO_DIR="$(cd -- "$(dirname -- "$(realpath "$0")")/.." && pwd)"
LOCAL_CONFIG="$MONO_DIR/watod-config.local.sh"
PAD_NAMES='Wireless Controller|DualSense Wireless Controller|DUALSHOCK'
SCAN_SECONDS=${SCAN_SECONDS:-25}

# shellcheck disable=SC1090
[[ -f "$LOCAL_CONFIG" ]] && source "$LOCAL_CONFIG"
CONTROLLER_MAC=${CONTROLLER_MAC:-}

usage() {
  sed -n '/^#   watod controller/s/^#   //p' "$0"
}

need_bluetoothctl() {
  command -v bluetoothctl >/dev/null || { echo "bluetoothctl not found (install bluez)"; exit 1; }
}

save_mac() {
  local mac=$1
  touch "$LOCAL_CONFIG"
  if grep -q '^export CONTROLLER_MAC=' "$LOCAL_CONFIG"; then
    sed -i "s/^export CONTROLLER_MAC=.*/export CONTROLLER_MAC=\"$mac\"/" "$LOCAL_CONFIG"
  else
    printf '\n# Bluetooth gamepad (watod controller)\nexport CONTROLLER_MAC="%s"\n' "$mac" >> "$LOCAL_CONFIG"
  fi
}

# Run several bluetoothctl commands in one session (the pairing agent only lives for one session)
bt_session() {
  local wait=$1
  shift
  { for cmd in "$@"; do echo "$cmd"; sleep 1; done; sleep "$wait"; echo quit; } | bluetoothctl >/dev/null 2>&1 || true
}

is_paired() {
  bluetoothctl info "$1" 2>/dev/null | grep -q 'Paired: yes'
}

is_connected() {
  bluetoothctl info "$1" 2>/dev/null | grep -q 'Connected: yes'
}

joystick_devices() {
  ls /dev/input/js* 2>/dev/null || true
}

find_pad() {
  # Prints the MAC of the first gamepad bluetoothctl has seen
  bluetoothctl devices 2>/dev/null | grep -E "$PAD_NAMES" | awk '{print $2}' | head -1
}

wait_for_joystick() {
  for _ in $(seq 1 10); do
    [[ -n "$(joystick_devices)" ]] && return 0
    sleep 1
  done
  return 1
}

cmd_pair() {
  need_bluetoothctl
  bluetoothctl power on >/dev/null
  bluetoothctl pairable on >/dev/null

  # Forget the old pairing first: a pad paired to another machine (or re-flashed host)
  # often fails to reconnect until both sides forget each other
  if [[ -n "$CONTROLLER_MAC" ]]; then
    echo "Removing the saved pairing for $CONTROLLER_MAC"
    bluetoothctl remove "$CONTROLLER_MAC" >/dev/null 2>&1 || true
  fi

  echo "Put the pad in pairing mode now:"
  echo "  PS4: hold SHARE + PS until the light bar double-flashes"
  echo "  PS5: hold CREATE + PS until the light bar flashes fast"
  echo "Scanning for ${SCAN_SECONDS}s..."
  bluetoothctl --timeout "$SCAN_SECONDS" scan on >/dev/null 2>&1 || true

  local mac
  mac=$(find_pad)
  if [[ -z "$mac" ]]; then
    echo "No gamepad found. Check it is flashing in pairing mode and run this again."
    exit 1
  fi
  echo "Found $(bluetoothctl info "$mac" | sed -n 's/^\s*Name: //p') at $mac, pairing..."

  bluetoothctl remove "$mac" >/dev/null 2>&1 || true
  bt_session 6 "agent NoInputNoOutput" "default-agent" "pair $mac"
  bluetoothctl trust "$mac" >/dev/null
  bt_session 4 "connect $mac"

  if ! is_paired "$mac"; then
    echo "Pairing failed. Put the pad back in pairing mode and run this again."
    exit 1
  fi
  save_mac "$mac"
  echo "Paired and trusted; saved CONTROLLER_MAC=$mac in watod-config.local.sh."
  echo "From now on, pressing PS reconnects it."
  if is_connected "$mac" && wait_for_joystick; then
    echo "Connected: $(joystick_devices | tr '\n' ' ')"
  else
    echo "Not connected yet: press PS on the pad (or run: watod controller connect)."
  fi
}

cmd_connect() {
  need_bluetoothctl
  [[ -n "$CONTROLLER_MAC" ]] || { echo "No saved pad; run: watod controller pair"; exit 1; }
  bluetoothctl power on >/dev/null
  echo "Connecting $CONTROLLER_MAC (press PS on the pad if it is asleep)..."
  bt_session 4 "connect $CONTROLLER_MAC"
  if is_connected "$CONTROLLER_MAC" && wait_for_joystick; then
    echo "Connected: $(joystick_devices | tr '\n' ' ')"
  else
    echo "Not connected. Press PS on the pad and try again, or re-pair: watod controller pair"
    exit 1
  fi
}

cmd_status() {
  need_bluetoothctl
  echo "Adapter: $(bluetoothctl show | sed -n 's/^\s*Powered: /powered /p' | head -1)"
  if [[ -n "$CONTROLLER_MAC" ]]; then
    local info
    info=$(bluetoothctl info "$CONTROLLER_MAC" 2>/dev/null || true)
    echo "Saved pad: $CONTROLLER_MAC ($(echo "$info" | sed -n 's/^\s*Name: //p'))"
    echo "  paired: $(echo "$info" | sed -n 's/^\s*Paired: //p')  trusted: $(echo "$info" | sed -n 's/^\s*Trusted: //p')  connected: $(echo "$info" | sed -n 's/^\s*Connected: //p')"
  else
    echo "Saved pad: none (run: watod controller pair)"
  fi
  local js
  js=$(joystick_devices)
  echo "Joystick devices: ${js:-none}"
}

cmd_forget() {
  need_bluetoothctl
  [[ -n "$CONTROLLER_MAC" ]] || { echo "No saved pad."; exit 0; }
  bluetoothctl remove "$CONTROLLER_MAC" >/dev/null 2>&1 || true
  [[ -f "$LOCAL_CONFIG" ]] && sed -i '/^# Bluetooth gamepad (watod controller)$/d; /^export CONTROLLER_MAC=/d' "$LOCAL_CONFIG"
  echo "Removed the pairing for $CONTROLLER_MAC and the saved MAC."
}

case "${1:-status}" in
  pair) cmd_pair ;;
  connect) cmd_connect ;;
  status) cmd_status ;;
  forget) cmd_forget ;;
  -h|--help|help) usage ;;
  *) usage; exit 1 ;;
esac
