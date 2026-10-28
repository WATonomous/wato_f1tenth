# CLAUDE.md

All agent instructions live in `AGENTS.md`, so every tool shares one source. Read it first.

@AGENTS.md

Skills in `.claude/skills/` wrap the shared workflows: `add-a-node`, `add-controller`,
`add-local-planner`, `port-to-jazzy`, `update-legacy-branch`, `car-bringup`, and `car-ssh` (run
commands on the Jetson over SSH from this machine).
