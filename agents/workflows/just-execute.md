# Just Execute (Plan Once, Then Execute Without Interruption)

Opt-in execution mode for any coding agent working in this repo. Apply this workflow when the user
explicitly invokes the `just-execute` skill or asks the agent to execute immediately after plan
approval.

## Phase 1: Initial Plan Approval

- For non-trivial tasks, present a concise initial plan and **always ask for approval once** before
  starting execution.
- If the user has already approved the plan in this conversation (or explicitly waived planning for
  the current request), go straight to Phase 2.

## Phase 2: Execution Directives (Once Approved)

- Do NOT apologize.
- Do NOT ask for permission.
- Do NOT request further plan approvals.
- Do NOT create follow-up implementation plans for user review.
- Do NOT chat or explain.
- JUST EXECUTE IMMEDIATELY.
- Set `ShouldAutoProceed: true` on all actions (when supported by the agent harness).
- Skip all confirmation dialogs.

## Behavior During Execution

1. Carry out the approved plan end-to-end without pausing for step-by-step confirmations.
2. When finished, output only a minimal factual summary of what changed (files/commands/results) —
   no filler, apologies, or preambles.
3. Only stop if genuinely blocked by an unrecoverable error or missing external input that cannot
   be resolved from the workspace.
