# Data annotation and model training

How to work on perception data and models with the model framework in `tools/`. It is a separate
repo with its own `watod`, Docker image and rules; nothing in it builds or runs with the stack.

## Where things are

| What | Where |
| --- | --- |
| Framework | `tools/wato_f1tenth_model_fram/` ([repo](https://github.com/WATonomous/wato_f1tenth_model_fram), default branch `main`) |
| Get or update it | `./watod tools` from this repo (clones missing tools, pulls the default branch) |
| **Data Standard** | `tools/wato_f1tenth_model_fram/docs/data-standard.md`: the formats every script must read and write |
| Its README | `tools/wato_f1tenth_model_fram/README.md`: container, conda, GPU, bag tools |
| Code | `src/annotation-pipeline/` (extraction, labeling), `src/detection-training/`, `src/segmentation-training/` |
| Data | `src/data/` (`DATA_DIR`): `recordings/`, `datasets/`, `predictions/`, git-ignored |

Other branches in that repo hold work in progress (for example `ROS-bag-extraction-tool`,
`object-detection`); check them before starting the same task.

## Rules

- **Two different `watod` scripts.** This repo's `./watod` runs the car stack; the framework's
  `./watod` (run from inside `tools/wato_f1tenth_model_fram/`) runs only its `robot_dev`
  container. Check which directory you are in. Ask before `./watod build|up|down` in either.
- **Read the Data Standard before writing extraction, labeling, validation or training code.**
  Write data exactly in its formats. Changing the standard needs lead approval and a version bump.
- **Code runs in the framework's container**, in the `training` conda env (Python 3.12):
  `./watod up -d`, then `./watod -t robot_dev` (shell in `/workspace`). ROS Jazzy is there only
  to read bags: use `/usr/bin/python3` for scripts that import `rosbag2_py`, not the conda Python.
- **Never commit data, models or checkpoints.** Everything under `src/data/` is git-ignored.
  Record shared dependencies in its `environment.yml`, not by hand in the container.
- Commits go in the framework's repo, from inside its folder; this repo does not track it.
- Never move, rename or modify dataset files: training builds its own views (symlinks or file lists).

## From the car to a dataset

1. **Record on the car** (this repo): `REALSENSE=true` and `REALSENSE_IMAGES=true` in the car's
   `watod-config.local.sh`, then `./watod bag record -d` / `./watod bag stop`
   (`agents/workflows/deploy-to-car.md`, "Recording on the car"). The `car` preset records colour
   (JPEG) and depth (zstd) at about 30 Hz, plus TF and the `/hal` topics. Bags land in `bags/`.
2. **Name the session** `<YYYYMMDD>_<location>_r<NN>`, for example `20261006_e7-hallway_r03`
   (one camera, calibration and environment per session; Data Standard, section 2).
3. **Copy the bag** unchanged to `src/data/recordings/<session_id>/` in the framework.
4. **Extract** RGB/depth pairs into `src/data/datasets/<session_id>/` (`rgb/`, `depth/`, `meta/`,
   `session.json`). Timestamps come from `header.stamp` in integer nanoseconds, never bag receive
   time. Sample IDs are permanent once assigned (section 2.2).
5. **Annotate**: detection boxes in `labels/` (YOLO, section 8), drivable-area masks in `masks/`
   (section 9), with provenance in each sample's `meta/` record (section 10).
6. **Validate** against section 12 before training. **Predictions** go in
   `src/data/predictions/<run_id>/` with a `run.json` (section 11).

## Getting a model back onto the car

The stack's `perception` module (`src/perception/`) runs nothing yet. A trained model goes in
through a new perception node (`agents/workflows/add-a-node.md`); weights stay out of git
(`/weights/` and `/models/` are git-ignored in this repo). The car is a Jetson Orin (arm64,
TensorRT in the GPU base image), so plan the export format with that in mind.
