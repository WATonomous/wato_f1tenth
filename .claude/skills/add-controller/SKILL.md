---
name: add-controller
description: Add a swappable controller to the F1TENTH action module (selected with CONTROLLER=<name>), so the human only writes the control law. Use when asked to add, write, port or plug in a controller or control algorithm (Stanley, MPC, LQR, pure pursuit variants).
---

# Add a controller

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/add-controller.md` and follow its checklist step by step.
3. Ask the human, if not given: the controller's name, C++ or Python, and whether it needs
   inputs beyond the interface (the scan, for example).
4. Start from `agents/templates/controller/`; don't write the wiring from scratch. If the human
   already has the algorithm, move it into `compute()` and leave the node alone.
5. Build only the new package, inside the dev container, never on the host
   (`agents/workflows/build-and-test.md`). Ask before any `./watod` or `docker` command.
6. Hand back: the files you created, the `CONTROLLER` name, the parameters with their YAML
   location, and what still needs a sim or car run.
