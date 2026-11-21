---
name: ml-data
description: Work on perception data and models with the model farm in tools/ (recording bags on the car, extracting RGB/depth samples, annotation, detection or segmentation training, predictions) following its Data Standard. Use when asked about datasets, labeling, annotation, bag extraction, training a detection or segmentation model, or the model farm (model framework).
---

# Data annotation and model training

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/ml-data.md` and follow it.
3. Run `./watod tools status`; if the farm is missing or behind, `./watod tools`.
4. Read `tools/wato_f1tenth_model_farm/docs/data-standard.md` before writing any extraction,
   labeling, validation or training code, and write data exactly in its formats.
5. The farm runs in its own conda environment (its `setup_env.sh`), not in Docker, and its GPU
   work runs on WATcloud: follow its `AGENTS.md` and ask before submitting a GPU job. Never commit
   data, models or checkpoints.
