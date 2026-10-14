# MPC / MPCC architecture conversation record

Compiled: 2026-10-02.

This is an organized compilation of the discussion, not a verbatim transcript.
Repeated messages are consolidated. It preserves the user's requirements,
corrections, proposals, research findings, and unresolved questions. No MPC
implementation or hardware benchmark was completed during this conversation.

The companion [living design document](mpcc-design.md) contains the more detailed
proposed interfaces and validation plan. Neither document is a finalized design.

**What the user wants to build**

Replace the current local planner and path-following controller with a joint
optimization-based planning and control system, likely MPCC. A global raceline
remains available as a reference.

The target computer is a Jetson Orin Nano 8 GB. It will also run perception,
localization, and the rest of the autonomy stack, so planning cannot assume exclusive
use of the CPU, GPU, or memory. Start with a CPU implementation and architect clear
boundaries so that GPU components, particularly TurboMPC, can be introduced later.

Design for moving opponents from the outset. Solo laps should use the same system
with no opponents, rather than requiring a separate controller architecture.
Initially consume a costmap, while preparing an interface that can later accept
object predictions with limited changes to application code.

The user wants costs and constraints with explainable meaning, minimizing arbitrary
gains. Vehicle system identification has not been done. The discussion should
challenge assumptions and develop a reference document as decisions are made.

The intended TurboMPC project is:

- [Toyota Research Institute implementation](https://github.com/ToyotaResearchInstitute/turbompc)
- [TurboMPC paper](https://arxiv.org/abs/2606.24039)

**Discussion progression and corrections**

1. The initial proposal separated perception adapters, vehicle modeling, the
   optimization formulation, solver backends, and a small maneuver-selection layer.
   The user accepted investigating a small decision layer, including the possibility
   of evaluating both passing sides with MPC or eventually using a learned proposal
   model.
2. The assistant initially cited an Orin Nano racing implementation operating at
   20 Hz for kinematic MPC and 10 Hz for dynamic MPC. The user challenged this:
   approximately 50 Hz is the desired control rate, and lower rates are undesirable.
   The correction was explicit: those published rates are one implementation's
   results, not an Orin Nano hardware ceiling.
3. The conversation adopted 50 Hz as the target for fresh MPC feedback updates.
   That corresponds to a 20 ms cycle, including the work needed to produce a valid
   command, rather than simply publishing an old command at 50 Hz.
4. The assistant proposed updating the active maneuver at 50 Hz while evaluating
   alternative overtaking maneuvers more slowly. The user explicitly accepted this
   architecture as worth investigating and asked whether others had done it.
5. The user mentioned an ideal speed around 10 m/s, then clarified that the number
   was arbitrary and actuator delay was the more important issue. It must not be
   treated as a hard requirement, acceptance threshold, or reason to select a model
   before measuring the car.
6. Discussion shifted to actuator dynamics, command delay, state age, and what the
   existing interface actually commands and measures.
7. The assistant supplied a broad list of questions for the user's lead. The user
   correctly pushed back that many answers could be found in code or standard build
   documentation. Further inspection resolved several questions, particularly the
   motor interface, command echo versus steering measurement, and inactive smoothing.
8. The remaining lead request was narrowed to pointers to the actual hardware list
   and authoritative deployed revision/configuration, including existing VESC exports
   and calibration records. These do not block continued architectural discussion.

The user also reported that transient question prompts disappeared before they could
read them. Subsequent questions were placed directly in the conversation so that
they remained visible. Researchable questions should be investigated before asking
the user to take them to another person.

**Current requirements and decision status**

| Item | Status |
| --- | --- |
| Jetson Orin Nano 8 GB, shared with the rest of the stack | Confirmed |
| CPU first, with a path to GPU acceleration | Confirmed |
| Replace local planning and path-following control | Confirmed |
| Global raceline available as reference | Confirmed |
| Moving opponents and solo laps in one architecture | Confirmed |
| Costmap first, prediction-capable interface | Confirmed |
| Fresh feedback optimization at 50 Hz | Design target; not demonstrated |
| Slower exploration of alternative maneuvers | Accepted for investigation |
| 10 m/s | Arbitrary aspiration, not a requirement |
| Identify actuator behavior and timing | Priority; measurements outstanding |
| MPCC | Leading formulation candidate, not finalized |
| acados versus TurboMPC CPU | Open; acados is the assistant's provisional preference |
| Dynamic versus kinematic vehicle model | Open; use measured prediction quality and operating envelope |
| Exact horizon, slow-layer rate, costs and margins | Open |

**Proposed modular architecture**

The main boundary for CPU/GPU substitution should be the optimization problem and
its solution. A backend owns numerical methods, derivative implementation, memory
and native warm-start data. Replacing individual linear-algebra kernels across
unrelated solver implementations is not assumed to be straightforward.

| Module | Responsibility |
| --- | --- |
| Input adapters | Convert ROS messages; align timestamps, frames and freshness |
| Reference | Track geometry, raceline, progress association, optional speed profile |
| Environment | Static occupancy, known/unknown space and time-dependent predictions |
| Maneuver generation | Distinct passing/following options, corridors and initial guesses |
| Vehicle model | Predict vehicle motion using identified parameters |
| Actuator model | Predict actual steering and propulsion response to available commands |
| Formulation | Define objectives, constraints and terminal behavior |
| Solver backend | Execute the CPU or GPU optimization |
| Validation and selection | Check feasibility and determine whether to adopt a solution |
| Runtime and command adapter | Schedule work, handle deadlines, convert commands and preserve arbitration |

These are logical/library boundaries; each does not need its own ROS node or
process. C++ runtime with Python model generation and experiments was proposed,
but the user's language preference remains unresolved.

Model changes that alter dimensions may require code generation, recompilation,
and warm-start reset. CPU/GPU substitution need not mean seamless switching during
a lap. CasADi and JAX implementations may share equations and validation fixtures
without sharing every line of executable model code.

**Fast control with slower maneuver exploration**

The fast loop re-optimizes the active maneuver from the latest ego state and valid
obstacle predictions every 20 ms. Maintaining a maneuver means retaining a passing
decision or corridor; it does not freeze steering, speed, braking or trajectory.

The background worker explores alternatives such as following, passing left and
passing right. It can use additional solve effort or a different horizon. Rates
such as 5, 10 or 20 Hz were suggested as benchmark points, not selected settings.

Distinct initial guesses alone may converge to the same passing side. Corridors
or equivalent constraints can preserve different maneuver choices. Multiple
opponents may require richer choices than one left/right pair. Maneuver continuity
and a justified switching rule are needed to avoid oscillation between near-equal
solutions.

The proposed handover contract is:

1. A background proposal records its source snapshot, prediction/reference/model
   versions, relevant object IDs, time origin, validity and intended handover time.
2. Before adoption, align it to the current state and command-application time.
3. Refine or re-solve using current obstacle constraints and actuator continuity.
4. Validate the resulting trajectory, including the transition and motion between
   prediction knots.
5. Compare on a consistent objective/time basis and switch atomically.

The fast loop must not block on background work. A late proposal is not automatically
valid, and raw costs from different horizons or old scenes are not directly
comparable. Separate solver workspaces and bounded queues were proposed. Threads
alone do not isolate CPU, memory bandwidth or GPU contention.

If the active maneuver becomes infeasible, the fast loop needs a timely contingency
response. It cannot wait for the slower search. The response's feasibility depends
on the state, available braking and surrounding traffic; no unconditional safety
guarantee was claimed.

A useful migration option is to retain CPU feedback control while accelerating
background candidate search on the GPU first, then evaluate whether moving the
active solve also helps.

**Evidence discussed**

| Source | Relevant finding | Limit |
| --- | --- | --- |
| Domingues et al., F1TENTH MPC | acados/HPIPM on Orin Nano; kinematic 20 Hz and dynamic 10 Hz in reported real experiments; accompanying repository specifies 8 GB | Does not establish maximum hardware capability or our obstacle-planning rate |
| Enrico et al., UAV NMPC comparison | Orin Nano processor-in-the-loop CPU NMPC reportedly below 15 ms at 50 Hz | Different task; exact memory variant and our concurrent workload not established |
| TurboMPC | CPU mode is available; reported racing experiment uses RTX 4070 Ti Super | Orin Nano compatibility/performance still needs validation |
| Raji et al., context-guided racing MPC | 20 Hz predictive planning with 100 Hz MPC on a full-scale racecar | Different hardware and planner/tracker architecture |
| TU Delft T-MPC++ | Optimizes distinct ways around moving obstacles using multiple local planners | Supports multiple-candidate reasoning, not proof of our asynchronous schedule |
| ForzaETH | Reports 10.21 m/s on a 150 m F1TENTH track, with maneuver and overtaking planning | Intel NUC and different control architecture; not proof of overtaking at that speed |

Sources:

- [Domingues paper](https://www.researchgate.net/publication/404404140_Real-Time_Model_Predictive_Control_for_High-Speed_Mobile_Robots)
- [Domingues implementation and hardware](https://github.com/duartetdomingues/f1tenth)
- [UAV NMPC/MPPI comparison](https://www.mdpi.com/2076-3417/15/16/9114)
- [TurboMPC paper](https://arxiv.org/html/2606.24039v1)
- [Dual-rate racing MPC](https://arxiv.org/html/2609.12660v1)
- [T-MPC++ authors' project](https://autonomousrobots.nl/paper_websites/topology-driven-mpc)
- [T-MPC++ implementation](https://github.com/tud-amr/mpc_planner)
- [ForzaETH paper](https://arxiv.org/html/2403.11784v2)

These sources support individual architectural choices. This conversation did not
establish a published demonstration of our complete proposed system on shared
Orin Nano 8 GB hardware.

**How to pursue 50 Hz**

Proposals included generated code, persistent workspaces, shifted warm starts,
bounded obstacle/candidate counts and acados SQP-RTI. RTI performs a limited
nonlinear optimization update per feedback cycle; feasibility and convergence
still need evaluation after abrupt changes.

The feedback period and prediction-grid spacing are separate choices. A 20 ms
feedback cycle does not require every future prediction interval to be 20 ms.
Nonuniform grids can preserve a useful lookahead with fewer stages, subject to
integration and collision-checking accuracy.

An illustrative allocation was 10 ms optimization, 4 ms preparation/validation/
publication, and 6 ms reserve. These are proposed engineering budgets, not measured
performance or hard real-time guarantees. Measure full-cycle latency, tails,
deadline misses, memory and thermal behavior with the rest of the stack running.

[acados features and RTI documentation](https://docs.acados.org/features/index.html).

**Costs and physical meaning**

The aim is a small, explainable set of choices, not a claim that tuning disappears.

- Progress or traversal time expresses performance.
- Footprint collision avoidance and track containment should be explicit constraints.
- Steering, acceleration, braking and combined tire capability should come from
  identified limits and model validity.
- Uncertainty allowances should have documented assumptions and measurement support.
- Raceline attraction, maneuver switching and numerical regularization remain
  preferences or numerical choices; units alone do not make them physical laws.

Standard MPCC's contouring/lag penalties and progress reward are not identical to
exact lap-time minimization. Comparing MPCC with explicit minimum-time formulations
remains open. Virtual progress must be coupled to actual motion so the optimizer
cannot gain reward without moving. Terminal behavior must account for the ability
to continue or brake beyond the visible optimization horizon.

The raceline's precise authority is unresolved: soft guidance across the full legal
track versus stronger adherence except during overtaking. Its optional speed profile
also needs a defined role rather than automatic exact tracking.

**Costmap now, predictions later**

Use a time-aware environment contract from the start, retaining separate static
geometry, observed/unknown space and predicted objects. Predictions should include
timestamps, footprints, validity and explicit uncertainty semantics.

A costmap adapter can initially provide stationary occupancy over a bounded
interval, but that is an assumption, not knowledge of object motion. Actual racing
around moving opponents needs prediction, a basic tracker, or a stated conservative
motion envelope. Synthetic predictions can support early development.

When prediction is introduced, avoid both retaining an object's old grid footprint
indefinitely and clearing it without replacement coverage. Prediction expiry,
disappearing objects and unknown space need explicit behavior. A longer horizon
cannot create observations outside sensor coverage.

**Actuator dynamics and timing**

The user clarified that these are more important than the arbitrary 10 m/s figure.
Distinguish state age, command dead time, actuator response dynamics and their
variability. A faster feedback loop does not remove physical actuator lag.

Model actual steering and propulsion response to the commands the interface accepts.
The proposed model boundary composes vehicle dynamics with actuator dynamics.
Initial conditions should include measured or estimated actuator state and the
history of commands that passed arbitration. Avoid treating requested steering as
an actual angle measurement or double-counting delay compensation.

A first-order steering response with separate dead time and rate limits was proposed
as an identification candidate, not an established description of this car.
Additional model complexity should improve prediction on held-out data.

[Research on actuator dynamics in MPC](https://arxiv.org/abs/1803.03478) provides
precedent, but does not provide this car's parameters.

**What repository inspection actually established**

The working checkout was `main`. The richer physical-car implementation was inspected
using `git show` on local branch `frenet-stateful-on-orin`, without switching branches.
Its inspected commit was `c3b40e25d0c1845bc0e99e5529208bd7eca665c6`.
The code deployed on the physical car was not independently checked.

| Finding | Evidence/location |
| --- | --- |
| Normal driving maps speed to ERPM and steering angle to servo position; acceleration fields are not consumed | `src/robot/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp` on the Orin branch |
| Driver supports additional current, duty-cycle and brake-current commands | `src/robot/vesc/vesc_driver/src/vesc_driver.cpp` |
| Zero requested speed maps to zero ERPM with inspected calibration; no automatic brake-topic switch | Ackermann converter and `bringup_robot/config/vesc.yaml` |
| Servo telemetry topic republishes the clipped command, not measured wheel angle | `VescDriver::servoCallback` and `sensors/servo_position_command` |
| Odometry can derive yaw rate from commanded steering | `vesc_ackermann/src/vesc_to_odom.cpp` |
| Motor telemetry and IMU are requested on a 20 ms timer | VESC driver; actual delivery rate unmeasured |
| Motor and IMU messages receive host timestamps on receipt | VESC driver; timestamps do not isolate acquisition delay |
| `jetson_pf.launch.py` does not launch the configured smoother | `bringup_robot/launch/jetson_pf.launch.py` |
| `jetson_up.launch.py` constructs a smoother Node but never adds it to the launch description | `bringup_robot/launch/jetson_up.launch.py` |
| EKF launch is commented out in the inspected PF launch | `jetson_pf.launch.py`; differs from runbook description |
| Reference track includes path and widths | Orin branch `global_planner/msg/ReferenceTrack.msg` |

The initial `main` inspection also found speed stored in `Path` position.z, a
50 ms pure-pursuit timer, and an ego-frame costmap with unknown cells. Costmap
publication timestamps and latest-TF lookup warrant attention for predictive control.

Source code establishes behavior of the inspected configuration. It does not prove
which physical components are installed, that the same revision is deployed, or
that no extra nodes/overrides or firmware limits exist.

**Default hardware versus the actual car**

The official build documentation describes a Traxxas Slash 4x4 Premium baseline
and retaining the chassis servo. The inspected IROS 2026 rules page, labelled
3.2026-draft, recommends hardware but allows variation and does not mandate one
servo model. The team's target event/final rules were not established.

Manufacturer servo speed describes shaft travel over 60 degrees at a specified
voltage. It is not a direct measurement of ROS-command dead time or the installed
steering assembly's dynamic response. Datasheets can provide starting information;
vehicle-specific system identification remains necessary.

- [Official BOM source](https://raw.githubusercontent.com/f1tenth/f1tenth_doc/master/getting_started/build_car/bom.rst)
- [Official chassis instructions](https://raw.githubusercontent.com/f1tenth/f1tenth_doc/master/getting_started/build_car/lower_level_chassis.rst)
- [Inspected competition rules](https://iros2026-race.roboracer.ai/rules.html)
- [Traxxas explanation of servo specifications](https://traxxas.com/articles/steering-servo-guide)

**Remaining questions and next work**

The broad lead questionnaire was superseded by two focused requests:

1. Where is the team's hardware list, including changes from the stock build?
2. Which deployed revision/configuration is authoritative, and where are any VESC
   settings exports or calibration records?

Further design work can continue while those remain open:

- Specify the actuator/state/timing contract and identification data requirements.
- Decide how commands at the real interface enter the prediction model.
- Define the time-aware environment and maneuver-proposal interfaces.
- Resolve raceline authority, track-boundary availability and prediction ownership.
- Compare representative CPU backends before committing to solver implementation.
- Benchmark active solves and background search under shared Orin workload.
- Define handover, stale-input, infeasibility and deadline-miss behavior.
- Select model fidelity, horizon and margins from measured evidence.

No CPU/GPU performance claims, steering model parameters, control gains, or race-speed
operating limits have been validated on this car during the discussion.
