# MPC prototype implementation and validation

Date: 2026-10-13. Implementation branch: `feature/mpc-prototype`, based on the locally inspected Frenet reference at `c3b40e25d0c1845bc0e99e5529208bd7eca665c6`. The original checkout remains on `mpc`. [Build/run instructions and interfaces](../src/robot/mpc_controller/README.md).

## Implemented scope

- One generated CPU acados SQP-RTI solver with persistent workspace, Cartesian kinematic bicycle motion, direct speed/steering requests, and prior-request memory.
- Existing periodic cubic Frenet reference and chord-length station, including its spline metric, locally seeded projection, heading checks, and lap wrapping.
- Frozen car-aligned solve frame initialized exactly at the latest measured rear-axle pose. No state-age propagation, filtering, actuator response, delay compensation, or uncertainty model.
- Native static costmap and reconstructed track intersection, conservative whole-body inflation, one bounded synchronous A* repair when needed, and one active corridor/trajectory.
- Independent analytic rollout, between-stage clearance checks, hard request/rate/terminal checks, and validation of the actual float32 command payload.
- ROS input freshness by observation and steady-clock receipt age, selected-request versioning, enable expiry, manual/emergency mux priorities, publication deadline gate, and an independent watchdog.
- Deterministic matching-model ROS simulation, offline multi-lap benchmarks, external AutoDRIVE wiring with required map/extrinsics arguments, and a stale-request watchdog in the simulator throttle bridge.

No moving-opponent tracking, background maneuver proposals, dynamic vehicle model, GPU path, full MPCC progress optimization, or physical-car trial was implemented.

## Environment and reproducibility

Measured on an x86-64 Intel Core i9-13980HX host, Ubuntu 24.04, ROS Jazzy, Python 3.12, CasADi 3.8.1, acados `988136c9ed5871246b9f25005a540600b2daf3d2`, with generic BLASFEO/HPIPM kernels and a Release build. Dependencies are under `$HOME/.local/opt/wato-mpc`; source/build artifacts are in the persistent worktree.

The configuration is `N=30`, `dt=0.04 s`, 50 Hz feedback, speed cap 1 m/s, wheelbase 0.3302 m, and inspected servo-derived asymmetric steering-request limits. These dimensions/limits are software assumptions, not validated vehicle parameters. All scenarios use the same model, weights, solver, and 0.1 m grid. The reference is a radius-10 m circle with 3 m left/right widths; a static 0.4 m by 1.2 m obstacle crosses the initial raceline, while the blocked scenario extends that obstacle across the grid.

Commands, exact fixtures, and acceptance checks are in the [README](../src/robot/mpc_controller/README.md), [benchmark](../src/robot/mpc_controller/tools/benchmark.cpp), and [tests](../src/robot/mpc_controller/test/test_controller.cpp). Each iteration represents 20 ms of plant time; the benchmark runs faster than wall time and refreshes the native scene every fifth iteration. It includes scene preparation, parameter updates, the solver call and extraction, validation, and command selection. It excludes ROS input callbacks, publication/DDS/mux transit, and the rest of autonomy. Some host compilation overlapped the benchmark run; workload, power, CPU affinity, and thermal state were not controlled. These are smoke measurements, not target-platform performance qualification.

## Offline closed-loop results

| Scene | Cycles | Approved | Stop/fallback cycles | Lap equivalents | Detected collisions / track violations | Minimum body-circle clearance | Maximum raceline offset |
|---|---:|---:|---:|---:|---:|---:|---:|
| Solo | 10,000 | 10,000 (100%) | 0 | 3.179 | 0 | 2.527 m | 0.00144 m |
| Static obstacle | 10,000 | 9,166 (91.66%) | 834 | 2.488 | 0 | 0.952 m | 1.577 m |
| Blocked | 2,000 | 0 | 2,000 | 0 | 0 | 2.529 m | 0 |

The obstacle was passed repeatedly; displacement/lap-progress assertions also passed. Its larger raceline offset includes the deliberate detour. Blocked scenes return `NoSeed` and hold zero requests; the search does not prove global infeasibility. Collision/track counts refer to these supplied deterministic fixtures and checks, not a general collision guarantee.

| Scene | p50 | p95 | p99 | p99.9 | Maximum | Observed >20 ms cycles | Peak process RSS |
|---|---:|---:|---:|---:|---:|---:|---:|
| Solo | 1.700 ms | 3.338 ms | 3.689 ms | 5.487 ms | 8.062 ms | 0/10,000 | 8,160 KiB |
| Static obstacle | 1.693 ms | 3.318 ms | 3.641 ms | 5.242 ms | 8.652 ms | 0/10,000 | 8,300 KiB |
| Blocked | 3.068 ms | 4.719 ms | 4.941 ms | 5.671 ms | 6.956 ms | 0/2,000 | 12,044 KiB |

The offline latency distribution is not the complete ROS control-cycle distribution. A rejected solve is an intervention even when a timely stop request is available. Obstacle-run rejections were 356 independent rollout-defect failures and 478 solver status/residual/version-gate failures. Thus this implementation does not establish fresh successful obstacle optimization every 20 ms, despite having no observed offline deadline misses. These failures remain visible; no tolerances or collision checks were removed to improve approval rate.

## Functional verification

Final checks: all six required ROS packages built successfully; all 12 prototype C++ tests, five live ROS integration tests, and 13 reference tests passed. Both installed launch descriptions loaded with `--show-args`. Dependency-script shell syntax and Python generation/test/launch syntax were checked.

The broader `local_planning` run passed 169 of 171 cases. Two failures were independently reproduced against unmodified `c3b40e25` source: `ManeuverBuilder.DefaultOvertakeAddsOneOffsetTailPerStationAndOffset` (4 tails versus expected 8) and `LocalPlanner.NoSafeLocalGeometryReportsExplicitUnavailableRecovery` (legacy recovery mode/index disagreement). Those unrelated legacy tests/code were retained. The prototype launch does not run that maneuver planner.

The final live ROS solo smoke window recorded 101 cycle metrics, 97 published approvals, p50 8.259 ms, p99 19.788 ms, maximum 20.149 ms, and combined controller/mux/fixture resident memory of 99,236 KiB. This includes ROS-side scene handling and command/path/status publication, but excludes subscriber delivery and the Python test runner's memory. It includes one elapsed sample over 20 ms and is too short to characterize tail latency. Observation count, gate rejections, and this overrun are retained rather than replaced by offline solver timing.

The standalone suite covers exact straight/turning bicycle integration, steering bounds, frame round trips, initial-state equality, request/rate/terminal bounds, float32 payload preservation, invalid values, unknown/out-of-grid/track rejection, rotated native-grid geometry, lap seams, expired scenes, corrupted solver states/versions, completed static detours, and an obstacle between clear stage endpoints. A 500-cycle solo regression requires <0.1 m maximum radial tracking error.

The shared reference-width regression first reproduced loss of a 0.1 m width notch between uniform lookup samples; retaining every original waypoint minimum fixes it. Boundary preparation also includes lookup knots and adjacent minima. A second regression reproduced 0.382 m solo error when line-of-sight reduction replaced a clear curved reference with long chords; preserving clear reference curvature produced the solo result above.

Live ROS tests launch actual controller/mux/fixture processes and check disabled startup, enabled motion, heartbeat loss, stale state and grid, recovery, manual and emergency priority, stalled solver rejection, paused and rewound simulation clocks, recovery after clock rewind, scan observation-time preservation and invalid rays/metadata, and stale throttle-request clearing. Publication gates recheck the job's own snapshot times and selected-command version. Review also caught and corrected the in-flight selected-request race and the distinction between observation age and receipt age.

## Remaining limits and measurements

The 5 cm chord-station boundary sampling with 5 mm tolerance is fixture-validated, not an analytic approximation bound for arbitrary tracks. New map alignment, global boundary simplicity, offset geometry, and reconstructed masks require independent validation. Square inflation, a covering circle, 0.20 m route padding, and rectangular corridors can reject feasible narrow passages. One route and one RTI step can stop in a feasible scene. Bounded scene/repair allocations remain; this is not a worst-case execution-time or allocation-free runtime.

External AutoDRIVE was wired but not run. Its bridge requests throttle proportional to speed; it does not enforce instantaneous speed. The bridge timeout clears stale throttle, but neither zero throttle nor zero requested speed establishes real braking. AutoDRIVE model mismatch, stopping, map identity, and extrinsics must be measured before enabling that launch.

No Jetson Orin Nano measurements, full perception/localization workload, sustained thermal test, physical parameter identification, or physical braking verification exists. The desktop numbers cannot be extrapolated into a verified 20 ms Orin budget. Next measurements should record release-to-command/DDS/mux timing, deadline misses, approval/intervention rates, RSS and peak allocations, temperature/clocks/throttling, prediction error against measured motion, tracking and swept clearance, with the intended autonomy stack running. Use identical scene recordings/configurations for comparisons; preserve invalid/stale-input, deadline, manual/emergency, collision, and track gates.
