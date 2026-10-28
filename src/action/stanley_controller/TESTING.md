# Stanley Controller — Testing

## Launch order

Stanley is started by the action module. In `watod-config.local.sh`:

```bash
export CONTROLLER="stanley"
export LOCAL_PLANNER="none"   # Stanley only follows the global path
```

Then on the host, with the AutoDRIVE simulator open:

```bash
./watod build action
./watod up
```

Foxglove comes up with the infrastructure module. Arm the dead man switch from the
gamepad (R1), or by hand inside a dev container:

```bash
ros2 topic pub --qos-durability transient_local --once \
  /hal/dead_man_switch std_msgs/msg/Bool "{data: true}"
```

Parameters live in `action_bringup/config/{sim,car}/stanley_controller.yaml`.
Debug topics are under `/debug/stanley/` (only with `enable_debug_vis: true`).

## Tuning

| Param | Default | Raise → | Lower → |
|---|---|---|---|
| `k_h` heading gain | 0.75 | react harder to heading error | calmer heading |
| `k_e` cross-track gain | 0.2 | pull onto line harder | less straight-line weave |
| `k_soft` low-speed softener | 0.5 | tame low-speed twitching/jerking | stronger CTE term |
| `closest_point_window` | 40 | denser path / higher speed | — |
