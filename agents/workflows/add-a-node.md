# Add a node

Checklist for adding a ROS 2 node or package so it works in sim and on the car without edits.

## 1. Decide where it goes

- A new feature (planner, controller, estimator): its own package under `src/robot/<name>`.
  After step 2, under the matching module: `src/world_modeling/`, `src/action/` or `src/interfacing/`.
- Sim-only glue (anything that reads `/autodrive/...`): `sim_util`, never a shared package.
- Car-only drivers: `interfacing` after step 2 (today on `orin`).
- Messages: a `<name>_msgs` package. Never reuse an upstream package name.

Look for an existing package that already does most of it before creating a new one.

## 2. Package skeleton

- C++: `ament_cmake`. Prefer `target_link_libraries(<target> <dep>::<dep>)` over
  `ament_target_dependencies` in new code, since the latter is deprecated upstream.
- Python: `ament_python`, no `tests_require` in `setup.py`.
- Declare every dependency in `package.xml`; the Docker build installs them with rosdep.
- Name executables `<thing>_node`, matching the existing packages.

## 3. Parameters

- Declare every tunable with a type and a neutral default:
  `declare_parameter<double>("speed_limit", 1.0)` in C++, `declare_parameter('speed_limit', 1.0)`
  in Python. Never declare without a default.
- Topic names are parameters too (`odom_topic`, `scan_topic`, ...), with defaults that follow
  the topic contract in `sim-to-car.md`.
- Put real values in YAML: today `src/robot/bringup_robot/config/<area>/<node>.yaml`; after
  step 3, `config/sim/` and `config/car/`. Key the YAML by the exact launched node name, or `/**:`.

## 4. Topics and QoS

- Subscribe and publish only contract topics: `/scan`, `/odom`, `/imu`, the localization
  output, `/drive/<source>`, `/global_planner/path`, `/dead_man_switch`.
- A new drive source publishes `AckermannDriveStamped` on `/drive/<source>`, and gets an entry
  with a priority and a timeout in the mux YAML. It never publishes to `/ackermann_cmd` directly.
- Debug and visualization topics: `rclcpp::QoS(1).best_effort()` (Python: `qos_profile_sensor_data`
  or a BEST_EFFORT profile), behind a rate limit or a `enable_debug_vis` parameter.
- Respect `/dead_man_switch` in anything that can move the car.

## 5. Launch

- Add the node to a launch file in `bringup_robot/launch/` with `parameters=[<yaml>]`.
- Sim: include it from `minimumEx.launch.py` or a launch file that includes it.
- Car: it will be included from `jetson_up` / `jetson_pf` once they are merged in step 2.
- After step 2: lifecycle nodes are listed in the bringup's lifecycle manager `node_names`.

## 6. Test and hand back

- Unit-test the logic that can run without ROS or the sim.
- Build only your package, inside the dev container, never on the host (`workflows/build-and-test.md`).
- In your summary, list the new topics and parameters and what still needs a sim or car run.
