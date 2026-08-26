# Docker
docker exec -it watod_wato-robot_dev-1 bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# MPPI
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mppi_cpp e7.launch.py

# Opponent Predictor
docker exec -it watod_wato-robot_dev-1 bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch opponent_predictor e7_opp_pred.launch.py

# Dedicated Cores
sudo taskset -cp 0-2 $(pgrep -f particle_filter)
sudo taskset -cp 3-5 $(pgrep -f mppi_node)
sudo taskset -cp 5   $(pgrep -f opponent_predictor)