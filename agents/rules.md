# Rules, with reasons

`AGENTS.md` lists these in one line each. This file explains why, so you can judge edge cases.

## Safety and infrastructure

1. **Ask before `./watod up|build|down`, `docker compose`, `docker push` or `git push`.**
   Builds take a long time on a laptop and longer on the Jetson. `up` may start a real car.
   Pushing publishes images or code the whole team pulls.
2. **Never run hardware commands.** `nvpmodel`, `jetson_clocks`, `taskset`, writes under `/dev`,
   and Bluetooth pairing change the physical car. Write the exact command for a human instead.
3. **Never edit `watod-config.sh` for personal settings.** Copy it to `watod-config.local.sh`,
   which is git-ignored. After step 2, CI rejects PRs that change `watod-config.sh`.

## Build only in the container (AGENTS.md rule 11)

**`colcon build`, `colcon test`, `ros2 launch` and `ros2 run` only run inside the dev
container.** The host has the wrong ROS distro, or none, and different dependencies, so a host
build proves nothing about the container or the car. It also drops `build/`, `install/` and
`log/` into the repo, and those then leak into the mounted workspace.

- Check where you are: `/.dockerenv` exists inside a container.
- On the host, write the commands for the human to run in the container, or ask before running
  them through `docker exec -it watod_<user>-<module>_bringup_dev-1 bash -c '...'`.
- Inside the container, build only the packages you changed (`workflows/build-and-test.md`).

## Sim and car

1. **No platform-specific values as code defaults.** On `orin`, pure pursuit's odometry topic,
   lookahead and speed limit were changed in C++, and the e-brake thresholds likewise. Each edit
   made merging back to `main` a conflict, and that is how sim and car ended up on separate branches.
   Declare the parameter with a neutral default, and set real values in
   `<module>_bringup/config/{car,sim}/*.yaml`.
2. **Shared nodes never subscribe to simulator topics.** Anything under `/autodrive/...` belongs
   only in `sim_util` (the sim adapter). See `workflows/sim-to-car.md`.
3. **`use_sim_time: false` on the car.** `jetson_pf.launch.py` currently runs the map server and
   lifecycle manager with `use_sim_time: True` on real hardware; do not copy that.
4. **(AGENTS.md rule 13) The contract lives under `/hal/`.** Before step 3, planners chose between
   `/odom`, `/pf/pose/odom` and `/ego_racecar/odom` per branch, and nobody could tell at a glance
   which topics were the hardware boundary. With one prefix, `ros2 topic list | grep /hal` shows
   the whole contract, and a node that reads only `/hal/*` runs on both platforms unchanged.

5. **(AGENTS.md rule 14) Algorithms are swapped by file name.** Before this, adding a controller
   meant hand-editing `action.launch.yaml`, its YAML, `package.xml` and three comments, and pure
   pursuit and MPPI were wired differently. Now `CONTROLLER=<name>` and `LOCAL_PLANNER=<name>`
   pick `action_bringup/launch/{controllers,local_planners}/<name>.launch.yaml`, and
   `action_bringup`'s registry test fails a half-registered one. Only the controller drives, so a
   planner that published to `/hal/drive/*` would fight it through the mux; `/global_planner/*`
   and `/local_planner/*` mirror each other so `ros2 topic list` shows who owns each path.

## ROS correctness

1. **Parameter YAML keys must match the launched node name.** `teleop_utils`'
   `joypad_config.yaml` is keyed `joypad_node:`, but the node launches as `gamepad_node`. ROS
   silently ignored the whole file. Use the exact node name, or `/**:` to match any node.
2. **Never name a package after an upstream package.** `src/wato_msgs/foxglove_msgs` is empty and
   has the same name as the real `foxglove_msgs`. Where it is installed, Foxglove cannot find the
   real definitions.
3. **New message packages must be visible to Foxglove.** Copy them into
   `docker/infrastructure.Dockerfile` and add them to `infrastructure_bringup/package.xml`
   (as done for `vesc_msgs`). Otherwise Foxglove and `ros2 bag` cannot decode the topics.
4. **Debug topics: BEST_EFFORT QoS and a rate limit.** With RELIABLE QoS, a slow remote
    subscriber (Foxglove over Wi-Fi) fills the publisher's history and blocks the callback. On MPPI
    this blocked the control loop and dropped ticks. Keep control topics (`/drive/*`) RELIABLE.
5. **Don't block the executor.** A long timer callback in a single-threaded executor starves
    subscriptions. MPPI hit this and now runs its control step from the pose callback, with a rate
    limit and a watchdog. Copy that pattern or use a separate callback group.

## Code and repo hygiene

1. **Stay in scope.** Change the package you were asked about. Infra changes (`watod`,
   `docker/`, `modules/`, CI) must change all coupled files together; see `roles/infra.md`.
2. **Match the surrounding code.** Same naming, comment density and idioms as the file you edit.
    Rename packages only as a planned change that updates every launch file and YAML.
3. **No build output, bags or secrets in git.** `build/`, `install/`, `log/`, `*.db3`, `*.mcap`,
    `.env`, tokens. Record bags with `./watod bag`, which writes to the git-ignored `bags/`.
4. **(AGENTS.md rule 12) No agent attribution in git.** Never add `Co-Authored-By: <agent>`,
    "Generated with <tool>" or similar to commit messages or PR descriptions, even if your tool
    does so by default. Commit with the human's configured `user.name` / `user.email` (check
    `git config user.email` first), so the change shows under their GitHub profile and the
    human stays the accountable author.
5. **Say what you could not verify.** Most changes need the sim or the car to test. State
    plainly what you checked (it builds, a unit test passes) and what still needs a human run.

## These docs

1. **`AGENTS.md` stays under 200 lines and only indexes.** Every agent loads it, so every extra
   line costs context in every session. Put detail in `agents/` and link it.
2. **Stay tool-agnostic.** Write plain Markdown that any agent or model can follow. Tool-specific
    files (`CLAUDE.md`, `.claude/`) only point back here.
