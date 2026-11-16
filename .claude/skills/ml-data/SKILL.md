---
name: ml-data
description: Work on perception data and models with the model framework in tools/ (recording bags on the car, extracting RGB/depth samples, annotation, detection or segmentation training, predictions) following its Data Standard. Use when asked about datasets, labeling, annotation, bag extraction, training a detection or segmentation model, or the model framework.
---

# Data annotation and model training

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/ml-data.md` and follow it.
3. Run `./watod tools status`; if the framework is missing or behind, `./watod tools`.
4. Read `tools/wato_f1tenth_model_fram/docs/data-standard.md` before writing any extraction,
   labeling, validation or training code, and write data exactly in its formats.
5. The framework has its own `./watod` (run from inside its folder). Ask before
   `./watod build|up|down` there too. Never commit data, models or checkpoints.
