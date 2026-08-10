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

uint8 CLEARANCE_FREE=0
uint8 CLEARANCE_SOFT_INFLATION=1
uint8 CLEARANCE_COLLISION=2
uint8 CLEARANCE_OUT_OF_GRID=3

uint8 requested_intent
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
bool projection_seed_was_stale
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
