---
name: just-execute
description: Get approval on the initial plan once, then execute immediately without apologizing, asking permission, chatting, or pausing for confirmation dialogs. Use when the user invokes /just-execute or asks to execute an approved plan uninterrupted.
---

# Just Execute

This skill wraps `agents/workflows/just-execute.md` so Claude Code, Antigravity, and other agents
share the same instructions.

1. Read and follow `agents/workflows/just-execute.md`.
2. **Always ask for approval on the initial plan first** (unless already approved).
3. **Once the initial plan is approved, execute uninterrupted:**
   - Do NOT apologize.
   - Do NOT ask for permission.
   - Do NOT request further plan approvals or create additional plans for review.
   - Do NOT chat or explain.
   - JUST EXECUTE IMMEDIATELY.
   - Set `ShouldAutoProceed: true` on all actions.
   - Skip all confirmation dialogs.
4. Return only a brief factual result when finished.
