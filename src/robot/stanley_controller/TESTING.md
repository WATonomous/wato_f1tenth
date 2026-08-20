# Stanley Controller — Testing

## Launch order

Source each new shell first

1. **Sensors + drivers**  — `ros2 launch bringup_robot minimumEx.launch.py`
2. **global_planner** — `ros2 run global_planner global_planner_node`
3. **Stanley** — with the real odom topic: `ros2 run stanley_controlller stanley_controler`
4. **foxglove_bridge** — `ros2 run foxglove_bridge foxglove_bridge`
5. **Arm dead-man**:
   ```bash
   ros2 topic pub --qos-durability transient_local --once \
     /dead_man_switch std_msgs/msg/Bool "{data: true}"
   ```

## Tuning

| Param | Default | Raise → | Lower → |
|---|---|---|---|
| `k_h` heading gain | 0.75 | react harder to heading error | calmer heading |
| `k_e` cross-track gain | 0.2 | pull onto line harder | less straight-line weave |
| `k_soft` low-speed softener | 0.5 | tame low-speed twitching/jerking | stronger CTE term |
| `closest_point_window` | 20 | denser path / higher speed | — |




