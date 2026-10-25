---
name: add-a-node
description: Add a ROS 2 node or package to the F1TENTH stack so it runs unchanged in sim and on the car. Use when asked to create a new node, package, controller, planner or drive source.
---

# Add a node

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/add-a-node.md` and follow it step by step.
3. Read `agents/workflows/sim-to-car.md` for the topic contract before choosing topic names.
4. Read the closest existing package (for example `src/robot/pure_persuit`) and match its style.
5. Build only the new package, inside the dev container, never on the host
   (`agents/workflows/build-and-test.md`). Ask before any `./watod` or `docker` command.
6. Hand back: the files you created, the new topics and parameters with their YAML location,
   and what still needs a sim or car run.
