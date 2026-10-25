---
name: port-to-jazzy
description: Port one F1TENTH ROS 2 package from Humble to Jazzy (Ubuntu 24.04, Python 3.12) as part of the big_infra_update migration. Use when asked to port, upgrade or make a package build on Jazzy.
---

# Port a package to Jazzy

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/port-to-jazzy.md`. Check the "Package notes" table for this package's
   known risks, and whether it should be deleted rather than ported.
3. Read the package's `package.xml`, build files, sources, launch files and YAML before editing.
4. Work through the checklist for its language. Port one package per change.
5. Build only that package, inside the dev container, never on the host. Ask before any
   `./watod` or `docker` command.
6. Hand back: the checklist items you changed, any parameters moved into YAML, and what still
   needs a sim or car run to confirm behaviour.
