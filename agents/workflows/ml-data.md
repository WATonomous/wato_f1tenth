# Data annotation and model training

How to work on perception data and models with the model farm in `tools/`. It is a separate repo
with its own conda environment and rules; nothing in it builds or runs with the stack.

Status (2026-11-21): the conda setup and the Data Standard library (`farm validate`) are on the
farm's `conda-overhaul` branch; its `main` still has the old `watod`/Docker setup until that is
merged (`git -C tools/wato_f1tenth_model_farm switch conda-overhaul`). The WATcloud job tool and
the auto-labeller are planned, not built. If the farm's README disagrees, trust its code and flag
this file.

## Where things are

| What | Where |
| --- | --- |
| Model farm | `tools/wato_f1tenth_model_farm/` ([repo](https://github.com/WATonomous/wato_f1tenth_model_farm), default branch `main`; renamed from `wato_f1tenth_model_fram`, the old URL redirects) |
| Get or update it | `./watod tools` from this repo (clones missing tools, pulls the default branch) |
| **Data Standard** | `tools/wato_f1tenth_model_farm/docs/data-standard.md`: the formats every script must read and write |
| Its README and agent notes | `tools/wato_f1tenth_model_farm/README.md`, `AGENTS.md`: environment, code layout, WATcloud jobs |
| Data | `data/` (`DATA_DIR`): `recordings/`, `datasets/`, `predictions/`, git-ignored |

Other branches in that repo hold work in progress (for example `ROS-bag-extraction-tool`,
`object-detection`); check them before starting the same task.

## Rules

- **No containers, one `watod`.** The farm runs in a conda environment pinned to Python 3.12 (the
  same Python as ROS 2 Jazzy), created by its `setup_env.sh`. It has no `watod`: this repo's
  `./watod` only runs the car stack.
- **Bags are read without ROS**, with the pure-Python `rosbags` library, so the farm needs no ROS
  install on a laptop or on WATcloud.
- **GPU work runs on WATcloud as SLURM jobs**, following the farm's `AGENTS.md` (a job tool with
  presets and an idle watchdog is planned there). Ask before submitting a GPU job; never run heavy
  work on the login nodes.
- **Read the Data Standard before writing extraction, labeling, validation or training code.**
  Write data exactly in its formats. Changing the standard needs lead approval and a version bump.
- **Never commit data, models or checkpoints.** Everything under `data/` is git-ignored.
  Record shared dependencies in its `environment.yml` / `pyproject.toml`, not by hand.
- Commits go in the farm's repo, from inside its folder; this repo does not track it.
- Never move, rename or modify dataset files: training builds its own views (symlinks or file lists).

## From the car to a dataset

1. **Record on the car** (this repo): `REALSENSE=true` and `REALSENSE_IMAGES=true` in the car's
   `watod-config.local.sh`, then `./watod bag record -d -o <session_id>` / `./watod bag stop`
   (`agents/workflows/deploy-to-car.md`, "Recording on the car"). The `car` preset records colour
   (JPEG) and depth (zstd) at about 30 Hz, plus TF and the `/hal` topics. Bags land in `bags/`.
2. **Name the session** `<YYYYMMDD>_<location>_r<NN>`, for example `20261006_e7-hallway_r03`
   (one camera, calibration and environment per session; Data Standard, section 2).
3. **Copy the bag** unchanged to `data/recordings/<session_id>/` in the farm.
4. **Extract** RGB/depth pairs into `data/datasets/<session_id>/` (`rgb/`, `depth/`, `meta/`,
   `session.json`). Timestamps come from `header.stamp` in integer nanoseconds, never bag receive
   time. Sample IDs are permanent once assigned (section 2.2). Depth must be aligned to colour
   (section 4): the driver's aligned topic when the bag has it, otherwise extraction aligns it.
5. **Annotate**: SAM3 pre-labels reviewed in CVAT. Detection boxes go in `labels/` (YOLO,
   section 8), drivable-area masks in `masks/` (section 9), with provenance in each sample's
   `meta/` record (section 10).
6. **Validate** against section 12 before training. **Predictions** go in
   `data/predictions/<run_id>/` with a `run.json` (section 11).

## Getting a model back onto the car

The stack's `perception` module (`src/perception/`) runs nothing yet. A trained model goes in
through a new perception node (`agents/workflows/add-a-node.md`); weights stay out of git
(`/weights/` and `/models/` are git-ignored in this repo). The farm and the stack share Python
3.12, so inference code from the farm can be installed into the stack's Python. The car is a
Jetson Orin (arm64, TensorRT in the GPU base image), so plan the export format with that in mind.
