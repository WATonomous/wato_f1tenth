# Frenet port — decisions, deviations, and things to check on a bag

Branch: `frenet-stateful-planner`, off `stateful-planner`.
Net **1093 lines removed** (1572 added, 2665 deleted, 42 files). All 125 unit
tests across 12 suites pass; clean `-Wall -Wextra -Wpedantic` Release build.

This records everything I decided rather than derived, everything where the
plan turned out to be wrong, and everything whose behaviour changed in a way a
bag will show. It is not a summary of the port — the plan file has that.

---

## 1. Where the plan was wrong

### 1.1 `LOCAL_PLANNER_GENERATION_FAILURE` does not exist

The plan's verification section said to watch `solver_failed` and
`arc_length_rejected` counters go to zero. There are no such counters.
`GeneratedConnection.reject_reason` was **never consumed anywhere** — not by the
planner, not by diagnostics, not by the decision message. Its only readers were
the tests that are now deleted.

**Decision:** `RejectReason` is kept on `FrenetConnectionResult` (it is genuinely
useful when writing tests, and three of the new generator tests assert on it),
but nothing new was plumbed to publish it. Adding per-reason rejection counters
is a real gap, but it is a diagnostics feature that predates this port and
inventing it here would have been scope creep. **If you want the reject
breakdown on a bag, say so and it is a small follow-up** — the reasons are
already produced, they just stop at `connect()`.

### 1.2 `selected_max_abs_d_m` is the overshoot metric; `max_offset_deviation` is gone

The plan made `selected_max_offset_deviation_m` the primary before/after number.
It could not be: `max_offset_deviation_m` was only ever computed by
`sideAndDeviation()`, which only PASS and its recovery family called. For
OVERTAKE and MERGE it was hard-coded `0.0`. The plan's headline metric would
have read zero on exactly the intent it was meant to measure.

I first shipped both fields. On review the deviation field earned nothing:
**§2.6 removed it as a ranking key**, so it was pure diagnostics, and the
question it answered — did a displaced car tuck to the inner passing magnitude
or settle on the outer one — is answered more directly by `terminal_d_m`, which
§2.6 already names as the field to watch.

**Decision:** `max_offset_deviation_m` is **deleted** — from
`ManeuverCandidate`, from `PlannerDecisionData`, and from `PlannerDecision.msg`.
`sideAndDeviation()` collapses to `staysOnSide()`, which is the only thing any
caller used its result for. The always-`false` `allow_start_center` parameter
went with it.

`selected_max_abs_d_m` — worst `|d|` along the path, measured for every intent —
stays, and is the only overshoot metric on the wire.

**A correction to the reasoning that first justified it.** The earlier version of
this section claimed "a quintic connection cannot exceed its commanded offset,
so `max_abs_d` should equal `max(|passing_d|, |terminal_d|)`." That is only true
from a flat start boundary. `startBoundary()` sets `d'` from the measured heading
error and `d''` from the measured vehicle curvature, and a quintic leaving with
those pointed outward overshoots its endpoint — ordinary polynomial behaviour,
not a solver defect the port removes. So `max_abs_d` is *not* redundant with
`passing_d`/`terminal_d`: it is the one piece of executed geometry those two do
not imply, and a displaced, yawed car is exactly the case where it diverges —
which is also when `track_bounds_rejected` climbs. Read it that way rather than
as a bug detector. A clothoid bulging to 1.2 m on a 0.75 m target would still
have read 1.2.

**Cost.** It is accumulated inside the sampling loop in
`FrenetConnectionGenerator::generate()` and returned on
`FrenetConnectionResult`, with `connect()` and `appendOffsetTail()` raising a
caller-owned running maximum so multi-leg paths compose. The first version swept
the finished path a second time per candidate, inside the parallel OVERTAKE
worker; that sweep is gone.

**Caveat you need to know before measuring.** The field does not exist on the
clothoid baseline, so a literal before/after needs it cherry-picked onto
`stateful-planner` — and there, computing it means reinstating the Newton offset
recovery this port deletes. Two honest options:

1. Cherry-pick just the `max_abs_d` computation onto the baseline using
   `lateralOffsetAt()` (still present there). Most direct, an hour of work.
2. Compare on fields **already on the wire on both sides**:
   `max_abs_curvature_inv_m`, `collision_rejected`, `track_bounds_rejected`,
   `valid_candidate_count`. If the overshoot diagnosis is right, collision and
   track-bounds rejections fall and valid candidates rise, on the same bag with
   the same config. This needs no baseline changes at all.

I would do (2) first — it is free and it tests the actual claim (fewer good
paths thrown away), where `max_abs_d` only tests the mechanism.

`max_abs_d` also replaced `station_hint_fallback` in the `LOCAL_PLANNER_PROFILE`
line, since that counter measured a code path that no longer exists.

### 1.3 `dκ/ds` denominator

The plan wrote `/ (speed_sq * speed_sq * speed_sq)`. I checked this rather than
copying it: `dκ/dt = (cross' q − 3 cross dot) / q^(5/2)`, and `dκ/ds` divides by
`ds/dt = q^(1/2)`, giving `q^3`. **The plan was right.** Noting it because it
looks like a typo and the next person will suspect it too. Verified against a
circle (`κ' = 0` to 1e-4) in `test_frenet_connection_generator.cpp`.

---

## 2. Behaviour changes a bag will show

### 2.1 OVERTAKE candidate count halves

The `curvature_modes` axis is gone, exactly as planned. Measured in the tests:
a single-station, single-offset, single-heading OVERTAKE went 6 → 4 candidates;
the fuller case went 20 → 12. **Tail counts are unchanged** — tails never had a
curvature mode. `generated_count` on the wire will drop accordingly for
OVERTAKE, and that is the intended saving, not a regression.

### 2.2 A turned intermediate boundary no longer gets the constant-offset curvature

This is the subtlest change in the port and it is worth understanding.

`boundary()` used to request terminal curvature `κ_ref / (1 − d·κ_ref)`
*regardless of the heading offset*. But that is the curvature of the
constant-offset curve, and a boundary turned 0.15 rad away from the reference
tangent is not tangent to that curve. The old code was asking the G2 solver for
the curvature of a curve the path does not follow.

Now `d'' = 0` at the boundary and the curvature falls out of the full formula
with the `d'` terms live. On a 30 m circle at `d = 0.55`, `heading_offset = 0.15`
that is `0.034324` where the old value was `0.033955` — a few times 1e-4, and
consistently so. Where `d' = 0` (every tail boundary, every PASS/MERGE target)
the two agree **exactly**, which the tests assert to 1e-12.

I consider the new behaviour more coherent, but it is a change, and it is the
one most likely to show up as slightly different OVERTAKE geometry on a bag.

### 2.3 The path now starts at `ref(ego_s) + ego_d·n(ego_s)`, not at the measured `(x, y)`

Under the clothoid family the start boundary was the measured world pose. In the
Frenet formulation `(ego_s, ego_d)` **is** the start — the pose is reconstructed
from it. These agree to the projection's own tolerance because `ego_s`/`ego_d`
come from `project()` on that same pose, and `local_planner` passes them from
the same `TacticalState`, so they are consistent by construction.

**Consequence:** an inconsistent `(pose, ego_d)` pair is no longer a
representable input, so it can no longer be detected. The test named
`DenseSideValidationRejectsAnInconsistentMeasuredSide` tested exactly that — and
was **already vacuous**: it passed `ego_d = 0.30` against a default `fullWidthM`
of `0.40`, so `sideOf()` returned 0 and `pass()` bailed before any geometry ran.
It never exercised the side check at all.

I replaced it with two real tests: `PassCandidatesNeverCrossToTheFarSide` (sweeps
the ego heading and asserts the actual documented contract — never more than one
vehicle width to the far side — and that a hard cross-heading does get rejected)
and `PassDeclinesWhenTheCarIsOnTheLine`.

The dense side sweep is **still live and still needed**: with a non-zero start
`d'` the quintic is not monotone and genuinely can dip across before recovering.
It is only redundant for a start that is already tangent.

### 2.4 `merge()` takes `ego_d`

Signature change, forced by 2.3: the start boundary is Frenet, so the measured
offset is an input rather than something the world pose carried implicitly.
`local_planner` passes `state.ego_d`, which it already had.

### 2.5 MERGE's `uses_offset_tail` stays `false`

My first pass set this `true` for MERGE, since merge genuinely does append a
constant-offset (at `d = 0`) tail. The old code said `false`. I reverted to
`false` — changing what a published field means for an intent is not part of this
port, and `selected_offset_tail` is on the wire.

### 2.6 Ranking (as agreed)

Implemented exactly as settled: safety binary everywhere, OVERTAKE on
`(safety, |passing_d|, |terminal_d|, time)`, PASS and MERGE on `(safety, time)`.
Three hand-rolled loops became one comparator over a 4-tuple with an
intent-dependent key extractor. Exact ties still fall through to first-seen, so
enumeration order remains the final tiebreak, as before.

The accepted-knowingly consequence from the plan stands: `max_offset_deviation_m`
was the only key pulling a displaced car back toward `preferredOffset(ego_d)`.
**Watch `PlannerDecision.terminal_d_m` under PASS** for the car settling at the
outer `passing_d_magnitudes_m` instead of tucking to the inner one. If it is
real, the fix is a `|d − preferred|` key ahead of time — not a return to tiers.
(Since §1.2 also deletes the published deviation field, `terminal_d_m` is now
the *only* way to see this. That is deliberate: the deviation field described
the whole path, but the behaviour to watch for is about where the car settles,
which is the endpoint.)

---

## 3. Decisions I made without asking

### 3.1 The reference window is rebuilt unconditionally, once per entry point

The plan had `planner_node` own a per-cycle `ReferenceWindow`. That would have
meant threading it through `LocalPlanner` into every `ManeuverBuilder` entry
point and changing four public signatures.

Instead `ManeuverBuilder` owns it as a `mutable` member (matching the existing
`mutable station_hint_stats_` pattern it replaces) and rebuilds it at the top of
each of `overtake`/`pass`/`recover`/`merge`. **No staleness cache.** A rebuild is
~61 `sampleAtS` calls and at most two entry points run per cycle (PASS, then its
recovery family), so the worst case is ~122 spline evaluations — against the
hundreds of candidates the window then serves. Caching on `ego_s` would have
bought nothing measurable and introduced a real staleness bug the first time the
raceline is republished mid-run.

`parallelFor` reads the window concurrently, but only after it is built and never
while writing, so this is safe.

### 3.2 The generator appends into the caller's path

`generate()` writes `CurveSample`s directly into the destination vector rather
than returning its own, and truncates back to the original size on rejection.
This avoids a temporary vector plus a copy per leg, and makes `connect()`
trivial. A rejected connection provably leaves no samples behind — asserted in
three tests.

### 3.3 A zero-length tail is success, not failure

Found while fixing a test. `appendOffsetTail(dist = 0)` must be a no-op success:
the path already ends where the tail would start. My first version returned
`false`, which silently dropped MERGE's longest completion distance (`completion
== horizon_m`) and PASS recovery's non-preferred full-horizon offsets. The old
`appendReferenceCurve` accepted it because its `while` loop simply never ran.
**This was a real bug, caught by `MergeUsesEveryUniqueCompletionDistance...`
finding 2 candidates where 3 were expected.**

### 3.4 Arc length is trapezoidal, and cannot match a chord sum exactly

`s` accumulates `sqrt(A² + d'²)·ds_ref` trapezoidally. It does not equal a
hypot-accumulated chord sum to 1e-6 as the plan's test sketch assumed — it came
out 1.7e-5 apart over 5.87 m.

That gap is **not** an integration error. The reference spline is parameterised
by cumulative *chord* length, so `|r'(s)|` is only approximately 1; on a
720-point circle that is a ~3e-6 relative bias, which reproduces the observed
number almost exactly. This is a pre-existing property of `RacelineReference`
that the clothoid path shared. It is ~3e-6 relative, and the velocity profile
only differences `s` over 0.1 m steps, so nothing downstream can see it. Test
tolerance is 1e-4 with that reasoning written next to it.

### 3.5 `frenetSecondDerivativeForVehicleCurvature` ported verbatim

Including its chart-singular fallback (`return κ_vehicle − κ_ref`). That fallback
is unreachable in practice — the sampling loop rejects `CHART_SINGULAR` at the
same threshold — but it keeps the arithmetic finite. Round-trip verified to
1e-12 against the forward formula in `VehicleCurvatureInversionRoundTrips`.

### 3.6 Start heading guard at 1.5 rad

`startBoundary()` encodes the ego heading error as `d' = A·tan(θ)`, which blows
up at ±90°. I reject beyond 1.5 rad (86°) purely to keep the arithmetic finite —
policy stays with the generator's `max_path_angle_deg` (60°), which rejects far
earlier. This is a numerical guard, not a behavioural threshold.

### 3.7 `CandidateSelector` exposes one `select(intent, ...)`

Rather than keeping three public methods that now share an implementation. The
call site in `local_planner` passes `profile_intent`, not `state.intent`, so a
PASS rerouted to `MERGE_ALIGNMENT` ranks as a merge — which is what it is. Since
everything that is not OVERTAKE shares the `(safety, time)` key, this is the
correct discriminator.

### 3.8 Diagnostics: `station_hint_fallback` → `max_abs_d`

The `LOCAL_PLANNER_PROFILE` line carried a station-hint fallback percentage and a
throttled warning about a "~60x latency cliff". Both measured the Newton offset
recovery, which no longer exists. Replaced with `max_abs_d=avg/p95/max`. The
warning is deleted rather than replaced — there is no longer a slow path to warn
about.

---

## 4. Incidental fix, unrelated to the port

`local_planning_core` linked `Threads::Threads` as **PUBLIC**, which put it in the
exported CMake link interface without a matching `find_package`. Every downstream
consumer failed to configure:

```
The link interface of target "local_planning::local_planning_core"
contains: Threads::Threads, but the target was not found.
```

**`costmap_test_tools` has not built since commit `c37dce5`** (the worker-pool
commit — `git log -S"Threads::Threads"` confirms). Verified pre-existing by
stashing this branch's changes and reproducing.

Fixed by making it `PRIVATE`: the worker pool is an implementation detail behind
`src/core/worker_pool.hpp`, a private header, so it has no business in the export
interface. `costmap_test_tools` builds again.

---

## 5. Known-unclean things I did *not* touch

- **`sample_msgs` fails to build.** Stale symlink in `build/`
  (`failed to create symbolic link ... existing path cannot be removed`).
  Reproduces identically on a stashed baseline. Wants `rm -rf build/sample_msgs`.
- **Lint failures.** `colcon test` reports 21 flake8, 134 lint_cmake, and 20
  uncrustify failures for `local_planning`. **Identical counts on the stashed
  baseline.** They come from untracked `build/` and `install/` directories nested
  *inside* `src/robot/local_planning/`, which the linters walk. `ament_uncrustify
  src include test` reports only 2 divergences, both pre-existing
  (`src/ros/planner_node.cpp`, `test/test_raceline_reference.cpp`) and neither
  touched by this port. Everything I wrote is clean under uncrustify and cpplint.
  Worth deleting those nested directories, but that is not this change.
- **`LocalPlanProfile::terminal_projection_ms`** is still summarised into the
  profile line and still never written by anything. Pre-existing dead field.
- **`pass_transition_distances_m: 1.0`** remains close to infeasible on geometry
  alone, exactly as the plan noted. `κ_max ≈ 5.77·D/L²`, so at `L = 1.0` the
  three configured `d` magnitudes need 1.73 / 3.17 / 4.33 against a 1.74 cap.
  Only the smallest scrapes in. Unchanged by the port; worth revisiting when the
  enumeration is next tuned.

---

## 6. Config changes

`config/local_planner.yaml`:

- **removed** `max_arc_length_m: 12.0`
- **added** `max_path_angle_deg: 60.0` (value ported from
  `frenet-planner:local_frenet_lattice_planner.yaml`)
- rewrote the stale `seed_window_m` comment — it claimed `project()` dominates
  PASS cost, which stopped being true once the side check became a plain read of
  `sample.d` (see §1.2) and is now
  doubly wrong: nothing projects path samples at all. `seed_window_m` and
  `tangent_tolerance_rad` themselves stay; `racing_state_machine.cpp:66` still
  projects the measured ego pose, which is the one genuinely unknown station.

`CMakeLists.txt` also now forces `CMAKE_BUILD_TYPE=Release` when unset, so a
clean `colcon build` cannot silently produce `-O0` numbers.

---

## 7. Still outstanding

The port is code-complete and green. What has **not** happened is step 7 of the
plan — measurement. Nothing here has run on a bag, so every performance claim in
the plan (`candidate_generation_ms` and `cycle_time_ms` must both fall) is still
unverified.

When you do run it:

- `profiling_enabled: true`, `profiling_intent_filter: "OVERTAKE"`.
- Read the rate off the profile line, **not** the event log — those are throttled
  to 1 Hz and cannot be used to infer Hz.
- Restore `max_velocity_mps` to 7.7 for at least one run. At the 2.0 testing cap
  the friction limit is inert (it only binds above `κ = 2.45`, past the 1.74
  reject), so curvature quality is not observable at 2.0 m/s.
