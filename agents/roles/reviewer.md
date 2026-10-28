# Reviewer

How to review a change or PR in this repo. Report findings, most serious first, each with the
file, the line, and what goes wrong. Don't rewrite the change unless asked.

## Check every change for

1. **Sim/car leaks.** A shared node subscribing to `/autodrive/...`, or a topic, gain, threshold
   or path hardcoded as a code default instead of YAML (`rules.md` rules 4 and 5).
2. **Parameters.** Declared with a type and default; YAML keyed by the launched node name or `/**:`;
   new parameters added to the YAML that launch files actually load.
3. **Drive path.** New drive sources publish on `/hal/drive/<source>` with a mux entry (priority and
   timeout), respect `/hal/dead_man_switch`, and never publish `/hal/ackermann_cmd` directly.
4. **QoS and timing.** Debug topics BEST_EFFORT and rate-limited; control topics RELIABLE; no long
   blocking work in timer callbacks of a single-threaded executor.
5. **Car safety.** `use_sim_time: false` on car launch files; e-brake priority stays highest; no
   change that could leave the car moving when a node dies (check timeouts).
6. **Frames.** Uses `map`, `odom`, `base_link`, `laser` consistently; no new static transforms
   that conflict with the particle filter's `map -> odom`.
7. **Build hygiene.** Dependencies declared in `package.xml`; no build output, bags or secrets
   committed; no edits to `watod-config.sh` for personal settings.
8. **Scope.** The change does what was asked and nothing else. Infra changes include every
   coupled file (`roles/infra.md`).
9. **Tests and verification.** Testable logic has a unit test; the PR says what was run in sim or
   on the car, and what wasn't.

## Severity

- **Blocker:** could move the car unsafely, breaks the build, or breaks sim or car for everyone.
- **Should fix:** a sim/car leak, a wrong QoS, a missing parameter or test.
- **Nit:** style, naming, comments. Keep these few.

## Output

Lead with a one-line verdict (ready, ready after fixes, or not ready). Then list findings.
If something can only be confirmed on the car, say so instead of guessing.
