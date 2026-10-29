# Add a local planner

A local planner turns the global raceline, the pose and the scan into the path the controller
follows right now: around an opponent, or away from an obstacle. Read "How selection works" in
`add-controller.md` first; this file covers the planner's side.

## Local planner interface

- **Inputs:** `/hal/localization/odom` (pose, map frame), `/hal/odom` (speed), `/hal/scan`
  (subscribe with sensor-data QoS), `/global_planner/path` (latched: RELIABLE + TRANSIENT_LOCAL),
  `/global_planner/reference_track` (the same, with track widths), and `/costmap` (base_link
  occupancy grid; world_modeling starts it whenever `LOCAL_PLANNER` is set).
- **`/local_planner/path`** (`nav_msgs/Path`, RELIABLE): `frame_id` `base_link` (x forward,
  y left), starting at the car, target speed in each pose's `position.z`. Publish it at a steady
  rate while it applies. `./watod contract` checks the frame.
- **`/local_planner/active`** (`std_msgs/Bool`, RELIABLE + TRANSIENT_LOCAL): true while the
  controller should follow your path, false to hand back to the global raceline. Publish on
  change. Set it false when your inputs go stale.
- **Never** publish to `/hal/drive/*`. Only the controller drives; the registry test fails a
  local planner whose topic parameters point there.
- Debug and visualization topics: `/local_planner/vis/...` or `/debug/<name>/...`, BEST_EFFORT,
  rate-limited.

With `LOCAL_PLANNER` set, the controller's launch file gets `local_planner` and subscribes to the
local topics (`pure_pursuit` turns on its overtake mode). MPPI ignores local planners.

## Checklist

Templates: `agents/templates/local_planner/{cpp,python}` (package `my_local_planner`) and
`agents/templates/local_planner/bringup/`. They build as they are, and until `plan()` is written
they keep `/local_planner/active` false, so the controller follows the global raceline.

1. **Name it.** The package `<pkg>` (for example `frenet_planner`) and the `LOCAL_PLANNER` name
   `<name>` (for example `frenet`).
2. **Copy the template** to `src/action/<pkg>/`, then replace `my_local_planner` with `<pkg>` and
   `MyLocalPlanner` with the CamelCase name, in file contents and in file and directory names.
3. **Register it** in `action_bringup`:
   - `bringup/my_local_planner.launch.yaml` -> `launch/local_planners/<name>.launch.yaml`
   - `bringup/my_local_planner.yaml` -> `config/car/<pkg>.yaml` and `config/sim/<pkg>.yaml`
   - `<exec_depend><pkg></exec_depend>` in `action_bringup/package.xml`
4. **Write the planner** in `plan()`, its tunables in `Params` and both YAML files (C++: also the
   node's parameter declarations). Return no path when the raceline is fine as it is.
5. **Test** `plan()` with real cases: an empty track, an obstacle on the raceline.
6. **Build and test** the new package in the dev container, then `./watod test action action_bringup`.
7. **Sim check** (ask first; simulator open on the host): `LOCAL_PLANNER=<name>` with
   `CONTROLLER=pure_pursuit`, `./watod build action`, `./watod up`, then:
   - `ros2 topic hz /local_planner/path` and `ros2 topic echo /local_planner/active`
   - `./watod contract` passes (`/local_planner/path` in `base_link`, with the controller subscribed)
   - the car still laps with nothing in the way, and leaves the raceline only when it should
8. **Hand back** the files created, the parameters and their YAML, and what still needs a car run.

## Porting a legacy planner

The frenet planner on the old branches already speaks this protocol under old names:
`/local_path` -> `/local_planner/path`, `/overtake_ready` -> `/local_planner/active`, and its
map-frame copy `/local_path_map` -> `/local_planner/vis/path_map`. Rename them in its YAML, add the
launch file and YAMLs, and keep its node.
