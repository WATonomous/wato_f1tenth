# AGENTS.md

Instructions for AI coding agents working in this repo (Claude Code, Codex, Cursor, Aider,
OpenHands, open-weight models, and others). Plain Markdown only, so any tool can read it.
If your tool does not load this file automatically, read it before doing anything else.

This file is an index. Details live in `agents/`; open only the files your task needs.

## What this repo is

The WATonomous F1TENTH stack: ROS 2 software for a 1/10th-scale autonomous race car.
It runs in two places:

- **Sim:** a developer PC running the AutoDRIVE simulator natively, plus our ROS 2 containers.
- **Car:** an NVIDIA Jetson (Orin Nano Super or Orin NX) running the same containers on arm64.

Everything runs in Docker through `./watod`, a wrapper around `docker compose`.

## Where the repo is right now

The repo is in the middle of an infrastructure migration on the `infra_update` branch.

| | Today (this branch, before step 2) | Target (after step 2 of the migration) |
| --- | --- | --- |
| ROS 2 | Humble, Ubuntu 22.04 | Jazzy, Ubuntu 24.04 |
| Infra | old `watod`, modules `vis_tools robot sim samples` | `wato_monorepo_template` infra: `infrastructure interfacing world_modeling action sim` |
| Sim vs car | separate branches (`main` = sim, `orin` / `mppi_test` = car) | one branch; `watod` detects car or dev machine |
| Middleware | default DDS | CycloneDDS with a fixed `ROS_DOMAIN_ID` (no Zenoh) |

Migration steps: 1 agents directory (this) -> 2 catch up to the template -> 3 topic contract,
shims and platform detection -> 4 JAX image -> 5 prebuilt multi-arch images.
If a doc in `agents/` disagrees with the code, trust the code and flag the doc.

## Hard rules

Each rule has its reason in `agents/rules.md`.

**Most important: never run `colcon build`, `colcon test`, `ros2 launch` or `ros2 run` on the host.**
They only ever run inside the dev container (rule 11). If `/.dockerenv` does not exist, you are on
the host: give the human the commands to run in the container, or ask before using `docker exec`.

1. Ask before running anything that starts, builds, stops or pushes infrastructure:
   `./watod up|build|down`, `docker compose`, `docker push`, `git push`.
2. Never run hardware commands (`nvpmodel`, `jetson_clocks`, `taskset`, anything touching `/dev`).
   Write them down for a human to run.
3. No topic names, gains, thresholds or file paths as C++/Python defaults that differ between sim
   and car. Put them in YAML under the bringup package's `config/`.
4. Never edit `watod-config.sh` for personal settings. Use `watod-config.local.sh` (git-ignored).
5. Never name a package after an upstream ROS package (the empty `foxglove_msgs` hid the real one).
6. YAML parameter keys must match the launched node name, or use `/**:`.
7. Debug and visualization topics use BEST_EFFORT QoS and a rate limit. Control topics stay RELIABLE.
8. Set `use_sim_time: false` for anything that runs on the car.
9. Keep changes inside the package you were asked to change. Infra changes follow `agents/roles/infra.md`.
10. Do not commit build output, bags (`*.db3`, `*.mcap`), maps you did not mean to add, or secrets.
11. Build, test and run ROS code only inside the dev container, never on the host.

## Index

Read `agents/architecture.md` first for anything non-trivial. Then pick what fits the task.

| File | Read it when |
| --- | --- |
| `agents/architecture.md` | You need the layout, packages, topics, TF tree or launch files |
| `agents/rules.md` | You want the reason behind a hard rule, or the full list |
| `agents/workflows/build-and-test.md` | You need to build, run or test something |
| `agents/workflows/add-a-node.md` | You are adding a node or a package |
| `agents/workflows/sim-to-car.md` | Code must behave the same in sim and on the car |
| `agents/workflows/deploy-to-car.md` | You are preparing something to run on the Jetson |
| `agents/workflows/port-to-jazzy.md` | You are porting a package from Humble to Jazzy |
| `agents/workflows/debug-mppi.md` | You are working on MPPI or the opponent predictor |
| `agents/roles/reviewer.md` | You are asked to review a change or a PR |
| `agents/roles/infra.md` | You are touching `watod`, Dockerfiles, compose files or CI |

Tool-specific helpers wrap these same files and add nothing new:

- Claude Code: `CLAUDE.md` imports this file. Skills live in `.claude/skills/`, permissions in
  `.claude/settings.json`.
- Codex and most other agents read this file directly.

## Quick reference (today's layout)

```bash
# on the host
cp watod-config.sh watod-config.local.sh   # personal settings go here
./watod build                              # ask first
./watod up                                 # ask first
./watod -t robot_dev                       # shell into the dev container (ask first)

# inside the container only (workspace: /home/bolty/ament_ws)
colcon build --packages-select <pkg> && source install/setup.bash
ros2 launch bringup_robot minimumEx.launch.py   # minimum sim stack
```

On the car, the main launch files are `jetson_up.launch.py` (drivers, teleop, mux, e-brake, EKF)
and `jetson_pf.launch.py` (all of that plus the particle filter, map server and global planner).
They live on the `orin` and `mppi_test` branches until step 2 merges them in.

## How to work here

- Read before you write. Open the package, its launch files and its YAML before changing it.
- Make the smallest change that does the job, and match the style of the surrounding code.
- Say what you could not verify. Most changes cannot be tested without the sim or the car.
- When you finish, list the files you changed and any command a human still needs to run.

## Maintaining these docs

- Keep this file under 200 lines. Add detail to a file in `agents/` and link it from the index.
- Keep each `agents/` file focused and under about 150 lines.
- When code changes what a doc describes, update the doc in the same change.
- Write for every agent: plain Markdown, no tool-specific syntax outside `.claude/`.
