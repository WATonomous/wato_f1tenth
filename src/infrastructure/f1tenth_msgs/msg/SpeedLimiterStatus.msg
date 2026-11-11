# ackermann_to_vesc's speed limiter (vesc_ackermann), on /control/speed_limiter at 10 Hz, best effort,
# car only: the speed asked for and the ramped speed actually sent to the VESC.
std_msgs/Header header

float32 target_speed      # m/s from the mux (/hal/ackermann_cmd); 0 once commands stop
float32 limited_speed     # m/s sent to the VESC after the limiter
float32 max_accel         # m/s^2 speeding up (0 = no limit)
float32 max_decel         # m/s^2 slowing down (0 = no limit)
bool active               # sending speed (a command arrived since the last stop)
bool stopping             # no command for cmd_timeout: braking to a stop
