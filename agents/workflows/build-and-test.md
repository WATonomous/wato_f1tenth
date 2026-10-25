# Build and test

How to build, run and test code. **Ask before running any `./watod` or `docker` command**
(`AGENTS.md` rule 1). If you cannot run commands, give the human the exact commands instead.

**Never run `colcon` or `ros2` on the host.** Every `colcon build`, `colcon test`, `ros2 launch`
and `ros2 run` below runs inside the dev container (`AGENTS.md` rule 11). If `/.dockerenv` does
not exist, you are on the host:

```bash
# from the host, with permission: run one command in the dev container
docker exec -it watod_<user>-robot_dev-1 bash -c \
  'cd /home/bolty/ament_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select <pkg>'
```

## Today (Humble, old `watod`)

### First-time setup (human)

```bash
cp watod-config.sh watod-config.local.sh
# in watod-config.local.sh:
#   ACTIVE_MODULES="vis_tools robot sim"
#   MODE_OF_OPERATION="develop"
./watod build
./watod up
```

The AutoDRIVE simulator runs natively on the host (see `setup-linux.md`). It connects to the
`sim` container on port 4567.

### Build inside the dev container

```bash
./watod -t robot_dev              # opens bash in watod_<user>-robot_dev-1
cd /home/bolty/ament_ws           # src/ is mounted here from the repo
source /opt/ros/humble/setup.bash
colcon build --packages-select <pkg>           # build only what you changed
colcon build --packages-up-to <pkg>            # when dependencies changed too
source install/setup.bash
```

Build only the packages you touched. A full `colcon build` is slow and rebuilds unrelated code.

### Run

```bash
ros2 launch bringup_robot minimumEx.launch.py   # required sim nodes (odom, scan, mux, converter)
ros2 run <pkg> <executable> --ros-args --params-file <yaml>
```

`minimumEx.launch.py` explains which nodes must be running in sim; read it before adding
launch files.

### Inspect

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /drive/autonomy --once
ros2 param dump /<node_name>
ros2 run tf2_tools view_frames
```

Foxglove connects to `ws://<host>:<uid*20>` (for example 20000 for uid 1000), not 8765.

### Tests

There is almost no test coverage today. Most packages only declare `ament_lint_auto`, and
`ackermann_mux` has upstream system tests. To run what exists:

```bash
colcon test --packages-select <pkg>
colcon test-result --verbose
```

When you add logic that can be tested without the sim (math, parsing, state machines), add a
unit test: gtest for C++, pytest for Python. Say in your summary what is still untested.

## Target (after step 2)

The template's `watod` replaces the commands above:

```bash
./watod build                     # build images for ACTIVE_MODULES
./watod up                        # start ACTIVE_MODULES + infrastructure
./watod test [module] [pkgs...]   # colcon test in each module's test image
./watod -t <module>_bringup_dev   # shell into a dev container (module listed as "<module>:dev")
./watod down
```

The rule stays the same: `colcon` and `ros2` run in containers, through `./watod test` or a dev
container shell, never on the host.

- Every node gets a Catch2 test through the template's `wato_test` helper (`add_wato_test` in CMake).
- Pre-commit runs clang-format, cpplint, ruff, shellcheck, hadolint, yamllint and markdown lint.
  Run `pre-commit run --files <changed files>` before handing work back.
- CI builds and tests only the modules a PR changes.

This section becomes the main one when step 2 lands; update this file in that change.
