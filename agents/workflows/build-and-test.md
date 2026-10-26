# Build and test

How to build, run and test code. **Ask before running any `./watod` or `docker` command**
(`AGENTS.md` rule 1). If you cannot run commands, give the human the exact commands instead.

**Never run `colcon` or `ros2` on the host.** Every `colcon build`, `colcon test`, `ros2 launch`
and `ros2 run` below runs inside a container (`AGENTS.md` rule 11). If `/.dockerenv` does not
exist, you are on the host.

## Configure (human, once)

```bash
cp watod-config.sh watod-config.local.sh
# in watod-config.local.sh, for sim:
#   export ACTIVE_MODULES="interfacing world_modeling action sim"
# for the car:
#   export ACTIVE_MODULES="interfacing world_modeling action"
#   export PLATFORM="car"
```

Add `:dev` to a module (`"action:dev"`) to get a dev container with `src/action` mounted.

## Build images

```bash
./watod build-base      # base images, built locally (see roles/infra.md); rerun when docker/base changes
./watod build           # source + deps stages, then each module image
```

`build-base` builds the CPU base plus the GPU base for this machine: CUDA 12.9 + TensorRT on
x86, CUDA 12.8 + TensorRT on the Jetson (run it there). Module images pick their base from
`docker/<module>.Dockerfile`.

## Run

```bash
./watod up              # ACTIVE_MODULES + infrastructure, detached; prints the Foxglove URL
./watod logs -f <service>
./watod down
```

In sim, start the AutoDRIVE simulator on the host (`setup-linux.md`) and press "Connect"; it
reaches the `sim` container's bridge on port 4567. Foxglove connects to `ws://<host>:<uid*20+2>`.

## Build and run code in a dev container

```bash
./watod -t action_bringup_dev           # with ACTIVE_MODULES="... action:dev"
# inside the container (workspace /ws, your module mounted at /ws/src/<module>):
colcon build --packages-select <pkg>    # build only what you changed
colcon build --packages-up-to <pkg>     # when dependencies changed too
source install/setup.bash
ros2 launch action_bringup action.launch.yaml
```

From the host, with permission, one command at a time:

```bash
docker exec -it watod_<user>-action_bringup_dev-1 bash -c \
  'cd /ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select <pkg>'
```

## Inspect

```bash
ros2 topic list
ros2 topic list | grep /hal          # the whole contract
ros2 topic hz /hal/scan
ros2 topic echo /hal/drive/autonomy --once
ros2 param dump /<node_name>
ros2 run tf2_tools view_frames
```

The containers use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ROS_DOMAIN_ID=0` by default; a
shell elsewhere needs the same values to see the topics.

## Tests

```bash
./watod test                    # every module
./watod test world_modeling     # one module
./watod test action pure_pursuit
```

Tests run in each module's `test` image with `rmw_fastrtps_cpp`. Lint-only tests were removed;
linting runs through pre-commit:

```bash
pre-commit run --files <changed files>
```

When you add logic that can be tested without the sim (math, parsing, state machines), add a
Catch2 test with `add_wato_test` (see `src/wato_test/README.md`). Say what is still untested.
