# CLAUDE.md

All agent instructions live in `AGENTS.md`, so every tool shares one source. Read it first.

@AGENTS.md

Skills in `.claude/skills/` wrap the shared workflows: `add-a-node`, `add-controller`,
`add-local-planner`, `port-to-jazzy`, `update-legacy-branch`, `car-bringup`, `car-ssh` (run
commands on the Jetson over SSH from this machine), `vesc-firmware` (VESC Tool, motor
settings and tuning on the car), `raceline` (the raceline generator in `tools/`),
`ml-data` (annotation and training with the model framework in `tools/`) and
`just-execute` (approve initial plan once, then execute immediately without explanations or confirmation dialogs).
