---
name: car-bringup
description: Prepare a step-by-step bring-up checklist for running the F1TENTH stack on the Jetson car (power mode, controller, containers, jetson_up / jetson_pf, checks). Use when someone is about to test on the real car or asks how to start the car.
---

# Car bring-up

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.
You prepare the checklist; a human runs every command on the car. Never run hardware commands
(`nvpmodel`, `jetson_clocks`, `taskset`, `sudo`) yourself.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/deploy-to-car.md`.
3. Ask which board (Orin Nano Super or Orin NX), which launch (`jetson_up` or `jetson_pf`), and
   whether MPPI is involved. If it is, also read `agents/workflows/debug-mppi.md`.
4. Produce a numbered checklist with the exact commands for that case, including the known traps
   that apply.
5. End with the checks to confirm the car is healthy before enabling autonomy.
