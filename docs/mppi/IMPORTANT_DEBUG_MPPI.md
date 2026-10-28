# IMPORTANT DEBUG MPPI

ros2 param dump /lmppi_node | grep -E "opponent_cost_enabled|opponent_behavior_mode|opponent_cost_weight|opponent_cost_radius|opponent_follow_weight|opponent_pass_weight|opponent_auto"

ros2 topic hz /drive
ros2 topic hz /mppi/optimal_trajectory
ros2 topic echo /mppi/debug/opponent_active
ros2 topic echo /mppi/debug/cost_opponent_sum
ros2 topic echo /mppi/debug/cost_opponent_follow_sum
ros2 topic echo /mppi/debug/cost_opponent_pass_sum
ros2 topic echo /mppi/debug/min_opponent_dist

