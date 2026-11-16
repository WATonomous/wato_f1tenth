---
name: raceline
description: Make, regenerate or fix a track's racelines (both driving directions) with the raceline generator in tools/, export them to race_tracks, tune the car profile or the edge margin, or plan grip/friction measurements. Use when asked about racelines, the raceline generator, the track editor, driving direction, track margins, or adding a track.
---

# Make or fix a raceline

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/raceline.md` and follow it.
3. Run `./watod tools status`; if the generator is missing or behind, `./watod tools`.
4. Make sure the conda env exists, creating it yourself if not:
   `conda env list | grep -q '^wato_raceline ' || tools/wato_f1tenth_raceline_generator/setup_env.sh`
   (run it in the background; it downloads packages). Then read the generator's own `AGENTS.md`.
5. Run `python make_raceline.py <track>` in the generator (in the background; it takes minutes). If it
   exits with code 2, the person must draw the edges in `track_editor.py`; never draw or invent them.
6. Show the person the reports (`outputs/<name>/report_*.png`) before `--export`, and remind them to
   check the raceline in Foxglove before the car drives it.
7. Commit generator changes in its own repo; commit only the exported `race_tracks` files here.
