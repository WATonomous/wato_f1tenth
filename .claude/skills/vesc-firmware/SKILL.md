---
name: vesc-firmware
description: Read, back up, change or test the VESC motor controller's settings on the car (motor PID, launch stutter, Max ERPM, braking, regen, battery cutoff), build VESC Tool on the Jetson, or run the headless VESC scripts. Use when asked about VESC firmware, VESC Tool, motor tuning, mcconf/appconf, or why the motor stutters, won't brake or is speed-capped.
---

# VESC firmware settings

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context.
2. Read `agents/workflows/remote-car.md`, then `agents/workflows/vesc-firmware.md`, and follow them.
3. Check access with `ssh -o BatchMode=yes -o ConnectTimeout=5 <host> true`. If it fails, ask the
   human about their SSH key or connection. Never ask for the car's password.
4. The stack must be down while VESC Tool holds the port: ask before `./watod down`.
5. Run `save_configs` before any write, and dry-run every `changes.json` first.
6. Ask before every script that moves the motor, saying whether it is off-ground or on the floor,
   and wait for the human's "go". One change per test.
