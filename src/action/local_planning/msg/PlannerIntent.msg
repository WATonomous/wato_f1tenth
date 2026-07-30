# Keep these constants in step with local_planning::PlannerIntent in
# include/local_planning/core/types.hpp.
uint8 FOLLOW_RACING_LINE=0
uint8 OVERTAKE=1
uint8 PASS=2
uint8 MERGE=3

std_msgs/Header header
uint8 intent
