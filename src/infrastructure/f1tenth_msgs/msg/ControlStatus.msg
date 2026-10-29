# Who has control of the car, published by drive_status (interfacing) at 10 Hz on /control/status.
std_msgs/Header header

# One word for the big status panel, from the input in control and the dead man:
#   AUTONOMY    the controller is driving (autonomy input, dead man on)
#   STOPPED     autonomy input, but the dead man is off: the controller holds the car stopped
#   MANUAL      the gamepad is driving (X)
#   KEYBOARD    the keyboard (Foxglove teleop) is driving
#   EBRAKE      the emergency brake has taken over
#   NO_COMMAND  no input at all: nothing reaches the motor
string state

string mode             # gamepad: MANUAL, AUTONOMY, BLOCKED, IDLE, NO_GAMEPAD; UNKNOWN if never heard
string active_input     # ackermann_mux input in control: joystick, keyboard, autonomy, emergency, none
bool dead_man           # /hal/dead_man_switch

float32 commanded_speed     # /hal/ackermann_cmd: what goes to the motor (m/s)
float32 commanded_steering  # rad
float32 measured_speed      # /hal/odom (m/s)

string controller       # CONTROLLER
string local_planner    # LOCAL_PLANNER
string track            # TRACK
