# ackermann_to_vesc's speed limiter (vesc_ackermann), on /control/speed_limiter at 10 Hz, best effort,
# car only: the speed asked for, the ramped speed sent to the VESC, and launch-guard / TC state.
std_msgs/Header header

float32 target_speed      # m/s from the mux (/hal/ackermann_cmd); 0 once commands stop
float32 limited_speed     # m/s sent to the VESC after the limiter
float32 max_accel         # m/s^2 configured top acceleration limit (0 = no limit)
float32 max_decel         # m/s^2 slowing down (0 = no limit)
bool active               # sending speed (a command arrived since the last stop)
bool stopping             # no command for cmd_timeout: braking to a stop
float32 effective_accel   # m/s^2 active accel limit after schedule, grip and cmd override
float32 measured_speed    # m/s wheel speed from VESC sensors/core (0 when unavailable)
bool lead_clamped         # closed-loop measured_speed + max_speed_lead capped limited_speed
bool stall_recovering     # commanding 0 ERPM briefly to clear VESC speed PID windup after a stall
bool stall_fault          # latched stop after too many consecutive launch stalls (clears at 0 cmd)
uint8 stall_count         # consecutive stall recovery attempts on the current launch
