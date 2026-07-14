uint8 SUCCESS=0
uint8 MISSING_INPUT=1
uint8 INVALID_REFERENCE=2
uint8 NO_PATH=3
uint8 DEADLINE_EXCEEDED=4
uint8 TF_FAILURE=5

std_msgs/Header header
uint8 status
uint8 intent
std_msgs/Header odometry_header
std_msgs/Header grid_header
std_msgs/Header reference_header
float64 runtime_ms
uint32 path_points
uint64 completed_attempts
uint64 overruns
builtin_interfaces/Time last_trajectory_stamp
