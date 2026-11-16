# The gamepad as gamepad_node (teleop_utils) sees it, on /control/teleop at 10 Hz, best effort:
# what each toggle is set to, so the Drive panel shows it (the buttons are toggles, invisible on the pad).
std_msgs/Header header

bool connected            # /joy heard within joy_timeout_s
string mode               # MANUAL, AUTONOMY, BLOCKED, IDLE or NO_GAMEPAD (as /control/mode)

bool manual               # X: manual control on (beats autonomy in the mux)
bool autonomy             # R1: the dead man switch toggled on
bool autonomy_blocked     # R1 on but blocked since manual was used: toggle R1 off and on
bool reverse              # O: reverse on (the throttle reverses at reverse_speed)
bool pit_limit            # square: forward speed capped at 1 m/s

float32 throttle          # throttle trigger travel, 0 released .. 1 fully pressed
float32 brake             # brake trigger travel, 0 .. 1
float32 steering          # steering command (rad)
float32 requested_speed   # m/s this node asks for while manual (0 otherwise)

float32 max_speed         # m/s at full throttle (joypad_config.yaml)
float32 reverse_speed     # m/s in reverse
