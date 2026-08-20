#!/usr/bin/env bash
#
# record_bag.sh - Record ROS 2 topics into an MCAP bag.
#
# Bags are saved in the directory you call this script from, named
# run_1, run_2, run_3, ... with the next number chosen automatically.
#
# Usage:
#   ./record_bag.sh
#
# Configure which topics to record below.
#
set -euo pipefail

# ============================================================
# CONFIG: set the topics you want to record here.
# Add one per line. Or set RECORD_ALL=true to record everything.
# ============================================================
TOPICS=(
    /bond
    /clicked_point
    /clock
    /commands/motor/brake
    /commands/motor/current
    /commands/motor/duty_cycle
    /commands/motor/position
    /commands/motor/speed
    /commands/servo/position
    /dead_man_switch
    /debug/lookahead_distance
    /debug/lookahead_point
    /diagnostics
    /drive/autonomy
    /drive/joystick
    /ekf/odom
    /goal_pose
    /initialpose
    /joy
    /joy/set_feedback
    /laser_status
    /map
    /map_updates
    /odom
    /parameter_events
    /pf/pose/odom
    /rosout
    /scan
    /sensors/core
    /sensors/imu
    /sensors/imu/raw
    /sensors/servo_position_command
    /tf
    /tf_static
    /local_frenet_lattice_viz
    /local_planner_all_candidates_viz
    /local_planner_projection_viz
    /local_planner_track_bounds_viz
    /local_planner_viz
    /local_path
    /local_path_map
    /occupancy_grid
    /overtake_ready
    /planner_decision
    /camera/camera/color/image_raw/compressed
    /camera/camera/color/camera_info
    /camera/camera/color/metadata
    /global_planner/path
    /global_planner/reference_track
    /global_planner/centerline
    /global_planner/vis
    /costmap
)

RECORD_ALL=false   # set to true to record ALL topics (ignores TOPICS)

# ============================================================
# CONFIG: performance / QoS
# ============================================================
# In-memory write buffer. Absorbs disk write bursts. "No bigger than 500 MB."
# 500 MB (decimal). Use 524288000 if you mean 500 MiB (binary).
MAX_CACHE_BYTES=$((1000 * 1000 * 1000))

# Subscriber queue depth for normal (non-latched) topics.
SUB_DEPTH=10

# Latched topics: published once at startup with transient_local durability.
# These get durability=transient_local so we still capture them; everything
# else gets volatile. Add any other transient_local topics you use here.
LATCHED_TOPICS=(
    /map
    /tf_static
    /global_planner/path
    /global_planner/reference_track
    /global_planner/centerline
    /global_planner/vis
    /local_planner_track_bounds_viz
    /overtake_ready
)

# ============================================================
# Save bags where the script was invoked from.
OUT_DIR="$(pwd)"

# Figure out the next run_# number by scanning existing run_* bags.
next_num=1
for d in "$OUT_DIR"/run_*; do
    [ -e "$d" ] || continue                      # skip if no matches
    num="${d##*/run_}"                           # strip path + "run_" prefix
    if [[ "$num" =~ ^[0-9]+$ ]] && (( num >= next_num )); then
        next_num=$(( num + 1 ))
    fi
done

BAG_NAME="run_${next_num}"
BAG_PATH="${OUT_DIR}/${BAG_NAME}"
QOS_FILE="${BAG_PATH}.qos.yaml"

# Build the set of topics we'll record (and generate QoS for).
if [ "$RECORD_ALL" = true ]; then
    mapfile -t TOPIC_SET < <(ros2 topic list)
    REC_ARGS=(-a)
else
    if [ "${#TOPICS[@]}" -eq 0 ]; then
        echo "No topics set. Edit the TOPICS list or set RECORD_ALL=true." >&2
        exit 1
    fi
    TOPIC_SET=("${TOPICS[@]}")
    REC_ARGS=("${TOPICS[@]}")
fi

# Is this topic in LATCHED_TOPICS?
is_latched() {
    local needle="$1" t
    for t in "${LATCHED_TOPICS[@]}"; do
        [ "$t" = "$needle" ] && return 0
    done
    return 1
}

# ============================================================
# Generate the QoS override file.
#
# reliability: best_effort  -> the recorder no longer ACKs messages, so
#   RELIABLE publishers are NOT throttled waiting on us. If the disk can't
#   keep up we drop messages instead of stalling your nodes. This is the
#   whole point: recording never holds anyone up.
#
# Latched topics keep transient_local so we still grab the one-shot message.
# They must stay reliable: transient_local's redelivery of the cached sample
# to a late-joining subscriber rides on the RELIABLE protocol's handshake in
# most RMW implementations, so best_effort + transient_local unreliably drops
# the one-shot message (this is why /map showed up empty in playback).
# best_effort adds no CPU; there is no compression anywhere in this script.
# ============================================================
: > "$QOS_FILE"
for t in "${TOPIC_SET[@]}"; do
    if is_latched "$t"; then
        cat >> "$QOS_FILE" <<EOF
$t:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
EOF
    else
        cat >> "$QOS_FILE" <<EOF
$t:
  history: keep_last
  depth: $SUB_DEPTH
  reliability: best_effort
  durability: volatile
EOF
    fi
done

echo "Recording to: ${BAG_PATH}  (storage: mcap)"
echo "QoS override: ${QOS_FILE}  (best_effort, no backpressure)"
echo "Write cache:  ${MAX_CACHE_BYTES} bytes"
echo "Topics:       ${REC_ARGS[*]}"
echo "Press Ctrl-C to stop."
echo

# -s mcap                        -> MCAP storage plugin
# -o PATH                        -> output bag directory
# --max-cache-size BYTES         -> in-memory write buffer (capped)
# --qos-profile-overrides-path   -> per-topic best_effort QoS
ros2 bag record \
    -s mcap \
    -o "${BAG_PATH}" \
    --max-cache-size "${MAX_CACHE_BYTES}" \
    --qos-profile-overrides-path "${QOS_FILE}" \
    "${REC_ARGS[@]}"
