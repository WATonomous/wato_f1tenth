# Add a controller

Controllers and local planners are swappable: you write the algorithm, and a fixed interface plus
a template handle the wiring. This file covers how selection works and how to add a controller.
For a local planner, read "How selection works" here, then `add-local-planner.md`.

## How selection works

`watod-config.local.sh` picks one of each, by file name:

```bash
export CONTROLLER="pure_pursuit"   # action_bringup/launch/controllers/<name>.launch.yaml
export LOCAL_PLANNER="none"        # action_bringup/launch/local_planners/<name>.launch.yaml
```

`action.launch.yaml` includes those two files and is never edited to add an algorithm. Each
file gets the args `platform`, `config_dir` (`action_bringup/config/<platform>`) and
`local_planner`. Swapping is a restart of the action container, not a rebuild:
`./watod up action`. A typo stops `watod` with the list of valid names.

| Role | Reads | Publishes |
| --- | --- | --- |
| Global planner (`world_modeling`) | the track's `raceline.csv` (`TRACK`) | `/global_planner/path`, `/global_planner/reference_track` (with track edge distances); latched, once |
| Local planner (`LOCAL_PLANNER`) | `/hal/localization/odom`, `/hal/odom`, `/hal/scan`, `/global_planner/path` | `/local_planner/path`, `/local_planner/active` |
| Controller (`CONTROLLER`) | `/hal/localization/odom`, `/hal/odom`, the paths, `/hal/dead_man_switch` | `/hal/drive/autonomy` only |

The `ackermann_mux` takes `/hal/drive/autonomy` at the lowest priority, so the gamepad and the
e-brake always win. Exactly one controller runs. Controllers that plan for themselves (MPPI,
MPC) are just controllers, run with `LOCAL_PLANNER=none`.

## Controller interface

- **Global path** `/global_planner/path` (`nav_msgs/Path`): map frame, a closed loop, target speed
  in each pose's `position.z`. Published once, latched: subscribe RELIABLE + TRANSIENT_LOCAL.
- **Local path** `/local_planner/path`: base_link frame, open, starts at the car, speed in `z`.
  Follow it only while `/local_planner/active` (`std_msgs/Bool`, latched) is true and the path is
  fresh; otherwise follow the global path. Subscribe only when `use_local_planner` is true (the
  launch file sets it from `LOCAL_PLANNER`).
- **Output** `/hal/drive/autonomy` (`AckermannDriveStamped`, RELIABLE). Publish a stop when the
  dead man switch is released, when there is no path, and when localization goes stale.
- Run the control step from the localization callback with a rate limit, not a long timer
  (rules.md, "Don't block the executor"). Debug topics: `/debug/<name>/...`, BEST_EFFORT, behind
  `enable_debug_vis`.

`pure_pursuit` follows this already (`overtake_ready_topic` is its name for the active flag).
`mppi` follows its own raceline CSV and ignores both paths.

## Checklist

Templates: `agents/templates/controller/{cpp,python}` (package `my_controller`) and
`agents/templates/controller/bringup/`. They build as they are and hold the car stopped until
`compute()` is written.

1. **Name it.** The package `<pkg>` (for example `stanley_controller`) and the `CONTROLLER` name
   `<name>` (for example `stanley`). Check `src/action/` for an existing package first.
2. **Copy the template** to `src/action/<pkg>/`, then replace `my_controller` with `<pkg>` and
   `MyController` with the CamelCase name, in file contents and in file and directory names.
   Fill in the `TODO` descriptions.
3. **Register it** in `action_bringup`:
   - `bringup/my_controller.launch.yaml` -> `launch/controllers/<name>.launch.yaml`
   - `bringup/my_controller.yaml` -> `config/car/<pkg>.yaml` and `config/sim/<pkg>.yaml`
   - `<exec_depend><pkg></exec_depend>` in `action_bringup/package.xml`
4. **Write the algorithm** in `compute()` and its tunables in `Params`, plus each new tunable in
   the node's parameter declarations (C++ only; Python declares every `Params` field) and both
   YAML files. Neutral defaults in code, real values in YAML (AGENTS.md rule 3).
5. **Test**: replace the template test's `TODO` with real cases for `compute()`.
6. **Build and test** only the new package in the dev container (`build-and-test.md`), then
   `./watod test action action_bringup`. The registry test there checks the launch args, both
   YAML files, YAML keys against node names, the `exec_depend`, and that topic parameters stay
   in their namespaces.
7. **Sim check** (ask first; the AutoDRIVE simulator must be open on the host):
   `CONTROLLER=<name>` in `watod-config.local.sh`, `./watod build action`, `./watod up`, then:
   - `ros2 topic hz /hal/drive/autonomy` shows the control rate while the dead man switch is held
   - `./watod contract` passes (it checks `/hal/drive/autonomy` has a publisher)
   - the car laps the sim track
8. **Hand back** the files created, the parameters and their YAML, and what still needs a car run.

## Porting an existing controller

Use the same steps, but move the algorithm into `compute()` (or keep its node and only add the
launch file and YAMLs). Rename its topics to the interface above with YAML parameters, not
remaps, so the registry test can see them.
