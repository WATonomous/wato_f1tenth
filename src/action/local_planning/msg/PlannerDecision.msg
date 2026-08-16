std_msgs/Header header

uint8 FOLLOW_RACING_LINE=0
uint8 OVERTAKE=1
uint8 PASS=2
uint8 MERGE=3

uint8 RELATIVE_NONE=0
uint8 BEHIND=1
uint8 OVERLAPPING=2
uint8 AHEAD_NOT_CLEAR=3
uint8 AHEAD_AND_CLEAR=4

uint8 NO_LOCAL_PATH=0
uint8 MANEUVER=1
uint8 BRAKING_FALLBACK=2
uint8 BRAKING_UNAVAILABLE=3

uint8 SOURCE_NONE=0
uint8 SOURCE_OVERTAKE=1
uint8 SOURCE_PASS_PREFERRED=2
uint8 SOURCE_PASS_RECOVERY=3
uint8 SOURCE_MERGE_ALIGNMENT=4
uint8 SOURCE_MERGE=5
uint8 SOURCE_BRAKING=6
uint8 SOURCE_MERGE_PROBE=7

uint8 TRANSITION_NONE=0
uint8 TRANSITION_FAST=1
uint8 TRANSITION_SLOW=2

uint8 REASON_NONE=0
uint8 REASON_OPPONENT_ENGAGED=1
uint8 REASON_PASS_BAND_ENTERED=2
uint8 REASON_OPPONENT_CLEARED=3
uint8 REASON_OPPONENT_CLEARANCE_LOST=4
uint8 REASON_RACELINE_COMPATIBLE=5
uint8 REASON_RACELINE_DEPARTED=6
uint8 REASON_OPPONENT_LOST=7
uint8 REASON_STATE_CORRECTION=8

uint8 RECOVERY_NONE=0
uint8 RECOVERY_MERGE_PATH_UNAVAILABLE=1
uint8 RECOVERY_OPPONENT_CLEARANCE_LOST=2
uint8 RECOVERY_BRAKING_FALLBACK=3
uint8 RECOVERY_NO_SAFE_LOCAL_PATH=4

uint8 CLEARANCE_FREE=0
uint8 CLEARANCE_SOFT_INFLATION=1
uint8 CLEARANCE_COLLISION=2
uint8 CLEARANCE_OUT_OF_GRID=3

uint8 requested_intent
uint8 proposed_intent
uint8 executed_intent
uint8 transition_class
uint8 transition_reason
uint8 recovery_reason
float64 pending_transition_s
uint32 pending_grid_count
uint32 merge_probe_valid_cycles
bool merge_probe_available
uint64 costmap_sequence
float64 costmap_stamp_s
uint64 opponent_observation_sequence
uint8 relative_position
bool opponent_detected
float64 opponent_gap_m

# Inputs to the FOLLOW_RACING_LINE / MERGE gate, and its result. The gate is a
# hard threshold with no hysteresis, so these say whether a transition was a
# real excursion or a cycle sitting on the limit.
# Station the gate believes the car is at. A wrong projection moves ego_d_m and
# heading_error_rad together, which reads exactly like a real excursion, so the
# station itself has to be on the wire to tell the two apart.
float64 ego_s_m
float64 ego_d_m
float64 heading_error_rad
bool raceline_compatible
uint8 executed_mode
uint8 candidate_source
bool selected_offset_tail
# Worst |d| on the selected path, measured for every intent. It is not implied
# by terminal_d_m: the connection leaves the measured d' and d'', so a displaced
# and yawed car swings past the offset it was commanded to reach.
float64 selected_max_abs_d_m
bool projection_seed_was_stale
# Weaker than the flag above: the projection stayed in the seed window, but only
# after widening the tangent tolerance. ego_s_m is still good; ego was yawed.
bool projection_heading_check_relaxed
uint8 clearance_class
float64 minimum_clearance_m
float64 max_abs_curvature_inv_m
float64 min_speed_mps
float64 max_speed_mps
float64 start_curvature_inv_m
bool start_curvature_from_steering
float64 terminal_d_m
float64 best_cost_s
float64 median_cost_s
uint32 generated_count
uint32 collision_rejected
uint32 out_of_grid_rejected
uint32 velocity_rejected
bool track_bounds_ready
float64 sustainable_left_m
float64 sustainable_right_m
uint32 track_bounds_rejected
uint32 valid_candidate_count
float64 cycle_time_ms
