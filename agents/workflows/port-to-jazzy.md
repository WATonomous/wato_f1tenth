# Port a package from Humble to Jazzy

Checklist for moving one package to ROS 2 Jazzy on Ubuntu 24.04 (step 2 of the migration).
Port one package per change, build it alone, and list what you could not verify.

## Before you start

- Check that every dependency in `package.xml` is released for Jazzy. Third-party packages the car
  uses are, except `vesc` and `range_libc`, which stay vendored. `transport_drivers` is released
  (`ros-jazzy-serial-driver`), so delete the vendored copy rather than porting it.
- Note what the package does today and which topics it uses, so you can check behaviour after.

## C++ (`ament_cmake`)

- [ ] Replace `std::experimental::optional` with `std::optional` (C++17), and similar `experimental` headers.
- [ ] Prefer `target_link_libraries(<target> PUBLIC <dep>::<dep>)` over `ament_target_dependencies`.
      It still works in Jazzy, but is deprecated upstream; convert when you touch the CMake anyway.
- [ ] Declare every parameter with a type and a default. No `declare_parameter("x")` without one.
- [ ] Move platform-specific defaults (topics, gains, thresholds, paths) into YAML (`rules.md` rule 4).
- [ ] Check the warnings on a clean build; Jazzy's compilers (GCC 13) flag more.

## Python (`ament_python`)

- [ ] Python is 3.12. `distutils` is gone; use `setuptools`. Remove `tests_require` from `setup.py`.
- [ ] `declare_parameter('x')` with no value: give each one a typed default. `particle_filter`
      has 21 of these.
- [ ] pip installs need `PIP_BREAK_SYSTEM_PACKAGES=true` (the template image sets it). Prefer the
      rosdep key in `package.xml` over pip when one exists.
- [ ] Native extensions: rebuild for 3.12. `range_libc` needs Cython < 3 and a `setuptools`
      build, with `sm_87` for the Jetson image and CPU-only elsewhere.
- [ ] numpy on Ubuntu 24.04 is 1.26; check code that relied on older APIs.

## Launch and config

- [ ] Launch files: replace removed or renamed arguments. Check `nav2` parameter names (Humble
      1.1 -> Jazzy 1.3) for the map server and lifecycle manager.
- [ ] YAML keys match the launched node name, or use `/**:`.
- [ ] `use_sim_time: false` for anything on the car.

## Verify

All of these run inside the Jazzy dev container, never on the host (`AGENTS.md` rule 11).

- [ ] `colcon build --packages-select <pkg>` passes with no new warnings you introduced.
- [ ] `colcon test --packages-select <pkg>` if tests exist.
- [ ] Launch the node alone and check its topics with `ros2 topic list/hz/echo`.
- [ ] Write down what needs the sim or the car to confirm, for example "drives the same lap".

## Package notes

The static fixes below were done in step 2 (`git log` on `big_infra_update`); the first real
build may still find more. Use this checklist for anything ported later.

| Package | Risk | Notes |
| --- | --- | --- |
| `particle_filter` + `range_libc` | High | Python 3.12, Cython < 3, untyped parameters |
| `vesc*` | Medium | `std::experimental::optional`, build against the released `serial_driver` |
| `ackermann_mux` | Low | Fix the old `nav_vel` / `joy_vel` default config |
| first-party C++ (`teleop_utils`, `safety_node`, `ackermann_ekf`, `global_planner`, `costmap`, `pure_pursuit`, `opponent_predictor`) | Low | Parameters into YAML, CMake targets |
| `mppi_example`, `mppi_cpp` | Deferred | Copy over; made runnable in step 4 |
| `gym_vis`, `action`, `foxglove_msgs`, `sample_msgs` | - | Delete, don't port |
