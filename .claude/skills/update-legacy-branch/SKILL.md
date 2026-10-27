---
name: update-legacy-branch
description: Bring the work on an old-layout F1TENTH branch (src/robot/..., Humble, pre-/hal topic names) onto the new layout, keeping every commit credited to its original author. Use when asked to update, port, rebase, revive or bring over an old or legacy branch, or a teammate's branch from before the migration.
---

# Update a legacy branch

This skill wraps the shared, tool-agnostic workflow. It adds no rules of its own.

1. Read `AGENTS.md` (hard rules) if it is not already in context. Rule 15 (authorship) has no
   exception except the owner/admin check in the workflow.
2. Read `agents/workflows/update-legacy-branch.md` and follow it step by step.
3. Run `agents/scripts/legacy_triage.py` first and show the human the whole report. Don't create
   a branch or replay anything until they have answered every decision in it.
4. Ask before `git switch -c`, the replay, any `git am --skip`, and every `./watod` or `docker`
   command. Never push, and never touch the old branch.
5. Hand back: ported commits and authors, what was dropped and why, the decisions made, and what
   still needs a sim or car run.
