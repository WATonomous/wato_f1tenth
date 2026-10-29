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

Steps 1-3 of the infrastructure migration are done (merged from `big_infra_update`): one
branch runs both the sim and the car on the `wato_monorepo_template` infrastructure.

| | Now | Still to come |
| --- | --- | --- |
| ROS 2 | Jazzy, Ubuntu 24.04 | |
| Infra | template `watod`, modules `infrastructure interfacing perception world_modeling action sim` | |
| Sim vs car | one branch; `watod` detects `PLATFORM` (car on a Jetson) and `GPU`; `/hal/*` contract | |
| Middleware | CycloneDDS, `ROS_DOMAIN_ID=0` (no Zenoh); loopback only on the car | |
| Algorithms | `CONTROLLER`, `LOCAL_PLANNER`, `TRACK` pick controller, local planner and track by name | |
| Images | built locally with `./watod build-base` | step 5: prebuilt multi-arch images on GHCR |
| MPPI / JAX | packages present, off by default (`CONTROLLER=mppi`), untested | step 4: JAX image |

Migration steps: 1 agents directory -> 2 catch up to the template -> 3 topic contract,
shims and platform detection -> 4 JAX image -> 5 prebuilt multi-arch images.
The old-layout branches (`orin`, `mppi_test` and older feature branches, tagged `legacy/*`)
are deprecated: bring their work over with `agents/workflows/update-legacy-branch.md`.
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
   and car. Put them in YAML under `<module>_bringup/config/car/` and `config/sim/`.
4. Never edit `watod-config.sh` for personal settings. Use `watod-config.local.sh` (git-ignored).
5. Never name a package after an upstream ROS package (the empty `foxglove_msgs` hid the real one).
6. YAML parameter keys must match the launched node name, or use `/**:`.
7. Debug and visualization topics use BEST_EFFORT QoS and a rate limit. Control topics stay RELIABLE.
8. Set `use_sim_time: false` for anything that runs on the car.
9. Keep changes inside the package you were asked to change. Infra changes follow `agents/roles/infra.md`.
10. Do not commit build output, bags (`*.db3`, `*.mcap`), maps you did not mean to add, or secrets.
    The one exception is the contract test bag in `src/sim/sim_bringup/test/data/`.
11. Build, test and run ROS code only inside the dev container, never on the host.
12. Never credit an agent in git: no `Co-Authored-By` trailers for an agent, no "Generated with"
    lines, in commits or PRs. Commits use the human's own git identity, so GitHub shows their profile.
13. Shared nodes talk to hardware and the sim only through the `/hal/*` contract (`/hal/scan`,
    `/hal/odom`, `/hal/imu`, `/hal/localization/odom`, `/hal/drive/<source>`, ...). New
    hardware-facing topics go under `/hal/`. See `agents/workflows/sim-to-car.md`.
14. Controllers and local planners are picked by file name (`CONTROLLER`, `LOCAL_PLANNER`); never
    edit `action.launch.yaml` to add one. Controllers publish only `/hal/drive/autonomy`; local
    planners publish only under `/local_planner/`. See `agents/workflows/add-controller.md`.
15. Work ported from another branch keeps its original author as the git author; the human porting
    it is added as `Co-authored-by`. No exception for anyone, unless `gh` shows the person asking
    is an owner or admin of the repo (`agents/workflows/update-legacy-branch.md`).

## Index

Read `agents/architecture.md` first for anything non-trivial. Then pick what fits the task.

| File | Read it when |
| --- | --- |
| `agents/architecture.md` | You need the layout, packages, topics, TF tree or launch files |
| `agents/rules.md` | You want the reason behind a hard rule, or the full list |
| `agents/workflows/build-and-test.md` | You need to build, run or test something |
| `agents/workflows/build.md` | You build images: which machine, which command, keeping rebuilds fast |
| `agents/workflows/add-a-node.md` | You are adding a node or a package |
| `agents/workflows/add-controller.md` | You are adding or swapping a controller, or need the planner/controller interface |
| `agents/workflows/add-local-planner.md` | You are adding or porting a local planner |
| `agents/workflows/sim-to-car.md` | Code must behave the same in sim and on the car |
| `agents/workflows/deploy-to-car.md` | You are preparing something to run on the Jetson |
| `agents/workflows/remote-car.md` | You run commands on the car over SSH (build, start, check topics, logs) |
| `agents/workflows/port-to-jazzy.md` | You are porting a package from Humble to Jazzy |
| `agents/workflows/update-legacy-branch.md` | You are bringing an old-layout branch onto the new layout |
| `agents/workflows/debug-mppi.md` | You are working on MPPI or the opponent predictor |
| `agents/roles/reviewer.md` | You are asked to review a change or a PR |
| `agents/roles/infra.md` | You are touching `watod`, Dockerfiles, compose files or CI |

Tool-specific helpers wrap these same files and add nothing new:

- Claude Code: `CLAUDE.md` imports this file. Skills live in `.claude/skills/`, permissions in
  `.claude/settings.json`.
- Codex and most other agents read this file directly.

## Quick reference

```bash
# on the host (ask before each of these)
cp watod-config.sh watod-config.local.sh   # personal settings: ACTIVE_MODULES, PLATFORM, ...
./watod build-base                         # local base images (once, and when docker/base changes)
./watod build                              # module images for ACTIVE_MODULES
./watod up                                 # start ACTIVE_MODULES + infrastructure
./watod test [module] [pkg...]             # colcon test in the module's test image
./watod contract                           # check the running stack against the /hal contract
./watod -t action_bringup_dev              # shell into a dev container (module listed as "action:dev")

# inside a dev container only (workspace: /ws)
colcon build --packages-select <pkg> && source install/setup.bash
```

With nothing set, `./watod up` runs the full stack for the machine: `interfacing world_modeling
action sim` on a PC (AutoDRIVE simulator on the host), `interfacing world_modeling action` on the
Jetson. The old `jetson_up` is `ACTIVE_MODULES="interfacing"`; the old `jetson_pf` is the car default.

Working on the car: never log an agent into the shared Jetson. Run the agent on your own machine
and send commands over SSH (`agents/workflows/remote-car.md`). If `ssh -o BatchMode=yes <host> true`
fails, ask the human to set up their SSH key; never ask for the car's password.

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
