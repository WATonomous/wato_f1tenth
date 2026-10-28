---
name: add-local-planner
description: Add a swappable local planner to the F1TENTH action module (selected with LOCAL_PLANNER=<name>), so the human only writes the planning algorithm. Use when asked to add, write, port or plug in a local planner, an overtaking or obstacle-avoidance planner, or a frenet planner.
---

# Add a local planner

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read "How selection works" in `agents/workflows/add-controller.md`, then
   `agents/workflows/add-local-planner.md`, and follow its checklist step by step.
3. Ask the human, if not given: the planner's name, and C++ or Python.
4. Start from `agents/templates/local_planner/`; don't write the wiring from scratch. A legacy
   planner that already publishes the old topics keeps its node: rename the topics in its YAML.
5. Build only the new package, inside the dev container, never on the host
   (`agents/workflows/build-and-test.md`). Ask before any `./watod` or `docker` command.
6. Hand back: the files you created, the `LOCAL_PLANNER` name, the parameters with their YAML
   location, and what still needs a sim or car run.
