---
name: car-ssh
description: Run commands on the Jetson car over SSH from this machine (build, start, check topics, read logs) without logging an agent into the shared car. Use when asked to do anything on the robot, the car, the Jetson or micro-orin.
---

# Work on the car over SSH

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/remote-car.md` and follow it.
3. Check access with `ssh -o BatchMode=yes -o ConnectTimeout=5 <host> true` before anything else.
   If it fails, ask the human about their SSH key or connection. Never ask for the car's password.
4. Ask before `./watod up|build|down` on the car. Hand any `sudo` or hardware command to the human.
5. Run long builds detached with a log (Bash `run_in_background`), and wait for the
   notification instead of polling.
