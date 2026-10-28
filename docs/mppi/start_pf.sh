source /opt/ros/humble/setup.bash
source install/setup.bash

export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1

ros2 launch bringup_robot jetson_pf.launch.py
