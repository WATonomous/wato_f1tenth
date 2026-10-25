# MPPI Stack — System Overview & Engineering Notes

A reference for the whole MPPI control + opponent-aware overtaking stack as it stands today, why it looks the way it does, and how to tune it.

This is structured so you can read it linearly to learn the system, or jump to a section as a reference during tuning.

---

## 0. TL;DR — the one-paragraph summary

The car runs an MPPI controller (Model Predictive Path Integral) implemented in JAX. A 40–60 Hz fixed-rate timer is the control clock; pose messages from particle-filter localization come in asynchronously and are cached. Each control tick: read latest cached pose → estimate vehicle state → look up reference trajectory along the raceline → sample $N$ candidate control sequences → roll each one forward through a single-track dynamic model → score them against costs (path tracking, wall clearance, opponent avoidance, optional slip/lat-acc) → take a reward-weighted average to get the control → publish `/drive`. An opponent-detection pipeline (LiDAR clustering → Kalman filter on raceline progress → predicted path) runs in parallel in C++ and feeds an opponent cost into MPPI. An auto-overtake state machine on top decides when to engage a pass and on which side.

---

## 0b. Mathematical formulation

### MPPI in one slide

Let $\mathbf{x}_t$ be the vehicle state (pose + velocities), $\mathbf{u}_t$ the control (steering rate, longitudinal accel), and $T = $ `n_steps` the rollout horizon.

For the current state $\mathbf{x}_0$, sample $K = $ `n_samples` control sequences:

$$
\mathbf{U}^{(k)} = \mathbf{a}_{\text{opt}} + \delta\mathbf{U}^{(k)}, \qquad
\delta\mathbf{U}^{(k)} \sim \mathcal{N}(0, \Sigma) \cdot \mathbf{1}_{[-1,1]}
$$

where $\mathbf{a}_{\text{opt}} \in \mathbb{R}^{T \times 2}$ is the **warm-start** (last solution shifted forward by one step), and the truncated normal keeps actions in $[-1,1]$.

Roll each sample forward through the dynamics $f$:

$$
\mathbf{x}^{(k)}_{t+1} = f(\mathbf{x}^{(k)}_t, \mathbf{u}^{(k)}_t), \qquad t = 0, \ldots, T-1
$$

Compute per-step reward $r^{(k)}_t = R(\mathbf{x}^{(k)}_t, \mathbf{u}^{(k)}_t)$ and accumulate from each step to the end (cost-to-go):

$$
R^{(k)}_t = \sum_{\tau = t}^{T-1} r^{(k)}_\tau
$$

Convert returns to **softmax weights** with a numerically-stable normalization:

$$
w^{(k)}_t = \frac{\exp\!\left(\frac{R^{(k)}_t - R^{\max}_t}{\lambda \cdot (R^{\max}_t - R^{\min}_t + d)}\right)}
                {\sum_{j} \exp\!\left(\frac{R^{(j)}_t - R^{\max}_t}{\lambda \cdot (R^{\max}_t - R^{\min}_t + d)}\right)}
$$

where $\lambda$ = `temperature` (smaller = winner-take-all) and $d$ = `damping` (numerical stability when all rewards are similar).

Finally, the new control sequence is the weighted mean of perturbations added to the warm-start:

$$
\mathbf{a}_{\text{opt}, t} \leftarrow \mathrm{clip}\!\left(\mathbf{a}_{\text{opt}, t} + \sum_{k} w^{(k)}_t \, \delta\mathbf{u}^{(k)}_t, \; -1, \; 1\right)
$$

The first action $\mathbf{a}_{\text{opt}, 0}$ is denormalized by `norm_params` and sent to `/drive`. Next tick, $\mathbf{a}_{\text{opt}}$ is shifted forward one step (the warm-start) and the cycle repeats.

### Reward function (per rollout step)

$$
\begin{aligned}
r_t = \;
& -\,w_{xy} \, \lVert \mathbf{p}^{\text{ref}}_t - \mathbf{p}_t \rVert_1 \\
& -\,w_v \, \lvert v^{\text{ref}}_t - v_t \rvert \\
& -\,w_\psi \left( \lvert \sin\psi^{\text{ref}}_t - \sin\psi_t \rvert + \lvert \cos\psi^{\text{ref}}_t - \cos\psi_t \rvert \right) \\
& -\,c_{\text{wall}}(\mathbf{p}_t) \\
& -\,c_{\text{opp}}(\mathbf{p}_t, \mathbf{p}^{\text{opp}}_t, \text{mode}) \\
& -\,c_{\text{slip}}(\beta_t) \;-\; c_{\text{latacc}}(v_t \cdot \omega_t) \;-\; c_{\text{steer}}(\delta_t) \\
& -\,1000 \cdot \mathbb{1}[\,\mathbf{x}_t \text{ non-finite}\,]
\end{aligned}
$$

### Cost terms

**Wall cost** — soft hinge on signed distance to nearest occupied cell:

$$
c_{\text{wall}}(\mathbf{p}) = w_{\text{wall}} \cdot \max\!\left(0,\; m_{\text{wall}} - d_{\text{SDF}}(\mathbf{p}) \right)^{p_{\text{wall}}}
$$

where $d_{\text{SDF}}$ is the precomputed Euclidean distance transform of the occupancy grid, $m_{\text{wall}}$ = `wall_cost_margin`, $p_{\text{wall}}$ = `wall_cost_power` (typically $2$).

**Opponent keep-out cost** (active only when `opponent_active` is true and mode allows):

$$
c_{\text{keepout}}(\mathbf{p}, \mathbf{p}^{\text{opp}}) =
w_{\text{opp}} \cdot \gamma^t \cdot \max\!\left(0,\; r_{\text{opp}} - \lVert \mathbf{p} - \mathbf{p}^{\text{opp}} \rVert\right)^{p_{\text{opp}}}
$$

with discount $\gamma$ = `opponent_cost_discount` over the horizon step $t$, radius $r_{\text{opp}}$ = `opponent_cost_radius`.

**Follow cost** (mode = follow, ego in same lane as opp):

$$
c_{\text{follow}} =
w_{\text{follow}} \cdot \gamma^t \cdot \mathbb{1}[\,|y_{\text{rel}}| \leq W\,] \cdot
\max\!\left(0,\; d_{\text{follow}} + x_{\text{rel}}\right)^2
$$

where $(x_{\text{rel}}, y_{\text{rel}})$ are ego–opp relative coordinates in the raceline-tangent frame, $W$ = `opponent_same_lane_width`, $d_{\text{follow}}$ = `opponent_follow_distance`.

**Pass cost** (mode = pass_left or pass_right):

$$
c_{\text{pass}} =
w_{\text{pass}} \cdot \gamma^t \cdot
\mathbb{1}[\,|x_{\text{rel}}| \leq W_{\text{long}}\,] \cdot
\max\!\left(0,\; \ell - s \cdot y_{\text{rel}}\right)^2
$$

where $\ell$ = `opponent_pass_lateral_offset`, $s = +1$ for pass-left and $-1$ for pass-right, $W_{\text{long}}$ = `opponent_pass_longitudinal_window`. The penalty fires when the ego rollout doesn't achieve the requested lateral offset on the chosen side within the longitudinal window around opp.

**Slip cost** (optional, soft hinge on sideslip $|\beta|$):

$$
c_{\text{slip}}(\beta) = w_{\text{slip}} \cdot \max(0,\; |\beta| - \beta_{\text{safe}})^2
$$

**Lateral acceleration cost** (optional):

$$
c_{\text{latacc}}(v\omega) = w_{\text{latacc}} \cdot \max(0,\; |v \cdot \omega| - a_{\text{safe}})^2
$$

**Steering saturation cost** (optional):

$$
c_{\text{steer}}(\delta) = w_{\text{steer}} \cdot \max(0,\; |\delta| - \rho \cdot \delta_{\max})^2
$$

with soft ratio $\rho$ = `steer_sat_soft_ratio`.

### Pose-delta state estimator (IIR)

For hardware (where PF twist is unreliable), each callback computes:

$$
\mathbf{v}^{\text{world}} = \frac{\mathbf{p}_k - \mathbf{p}_{k-1}}{\Delta t}, \qquad
\omega^{\text{pose}} = \frac{\psi_k - \psi_{k-1}}{\Delta t}
$$

Project into the body frame:

$$
v_x^{\text{pose}} = \cos\psi \cdot v_x^{\text{world}} + \sin\psi \cdot v_y^{\text{world}}, \qquad
v_y^{\text{pose}} = -\sin\psi \cdot v_x^{\text{world}} + \cos\psi \cdot v_y^{\text{world}}
$$

The longitudinal estimate fuses pose-derived, command, and previous estimate:

$$
\hat{v}_x = 0.55 \cdot \tfrac{1}{2}(v_x^{\text{pose}} + v_x^{\text{raw}}) + 0.30 \cdot v^{\text{cmd}} + 0.15 \cdot \hat{v}_x^{\text{prev}}
$$

The lateral and yaw-rate estimates are IIR-filtered with adaptive priors (the prior weight is multiplied by `state_est_hiccup_prior_scale` ≈ 0.35 when $\Delta t > $ `state_est_hiccup_dt` ≈ 0.06 s):

$$
\hat{v}_y \leftarrow (1 - \alpha_y) \cdot v_y^{\text{obs}} + \alpha_y \cdot \hat{v}_y^{\text{prev}}, \qquad
\hat{\omega}  \leftarrow (1 - \alpha_\omega) \cdot \omega^{\text{obs}} + \alpha_\omega \cdot \hat{\omega}^{\text{prev}}
$$

with $\alpha_y$ = `state_est_vy_prior` (default 0.4), $\alpha_\omega$ = `state_est_wz_prior` (default 0.4). Sideslip:

$$
\hat{\beta} = \arctan2\!\left(\hat{v}_y, \max(|\hat{v}_x|, \epsilon)\right)
$$

### Opponent KF (in opp_predictor)

State $\mathbf{z} = (s, v_s)$ along the raceline arc length. The **filter itself** uses a pure constant-velocity model:

$$
\begin{bmatrix} s \\ v_s \end{bmatrix}_{k+1} =
\begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} s \\ v_s \end{bmatrix}_k
+ \mathbf{w}, \qquad
\mathbf{w} \sim \mathcal{N}\!\left(0, \begin{bmatrix} q_s & 0 \\ 0 & q_v \end{bmatrix}\right)
$$

Measurement (projected detection + finite-difference progress speed):

$$
\mathbf{y}_k = \mathbf{z}_k + \mathbf{n}_k, \qquad
\mathbf{n}_k \sim \mathcal{N}\!\left(0, \begin{bmatrix} R_s & 0 \\ 0 & R_v \end{bmatrix}\right)
$$

Standard KF predict + update on this state.

**Published horizon — shape-preserving propagation against the raceline profile.** The N-step prediction is *not* simply constant-velocity propagation of the KF state, and it is *not* an absolute blend toward the raceline's reference speed either. Instead, the KF's measured velocity is the **anchor** at step 0, and at each future step we apply only the **delta** of the raceline reference speed over that step:

$$
s^{\text{pred}}_{k+1} = s^{\text{pred}}_k + v^{\text{pred}}_k \cdot \Delta t
$$

$$
v^{\text{pred}}_{k+1} = \max\!\left(0,\; v^{\text{pred}}_k + \beta \cdot \big(v^{\text{ref}}(s^{\text{pred}}_{k+1}) - v^{\text{ref}}(s^{\text{pred}}_k)\big)\right)
$$

with initial values $v^{\text{pred}}_0 = \hat{v}_s$ from the KF, $s^{\text{pred}}_0 = \hat{s}$, and $\beta$ = `profile_speed_blend` (default 0.7) when the detection is fresh, or $\beta$ = `out_of_sight_profile_speed_blend` (default 1.0) when stale.

**Why this form**: an opponent measured at $\hat{v}_s = 3$ m/s on a stretch where the raceline says 5 m/s isn't going to suddenly catch up to raceline pace; they'll *brake into the next turn* but stay roughly the same offset. With the shape-preserving form, the offset $(v^{\text{pred}} - v^{\text{ref}})$ is preserved exactly when β = 1 and partially when β < 1 — but the *shape* of the prediction (when to brake, when to accelerate) follows the raceline. β controls how strongly the raceline's per-step changes propagate into the prediction:

- $\beta = 0$ → pure constant-velocity from the KF, ignores the raceline entirely.
- $\beta = 1$ → preserve the measured KF–raceline offset exactly through the horizon; opponent brakes/accelerates 1-to-1 with the raceline reference.
- $\beta = 0.7$ (default) → opponent applies 70 % of the raceline's per-step velocity change.

The KF stays simple and noise-resistant; the prior knowledge "opponents on this track follow this profile's *shape*, but at their own pace" lives in the propagation layer.

There's also `use_profile_speed_fallback`: when the per-tick progress-speed measurement is invalid (NaN, negative, etc.), the value fed into the KF measurement update is replaced by $v^{\text{ref}}(s_{\text{meas}})$, so the filter re-anchors on the raceline expectation rather than discarding the update.

### Single-track dynamic model (used in MPPI rollouts)

State: $\mathbf{x} = (x, y, \delta, v_x, \psi, \omega, \beta)$. Inputs: $\mathbf{u} = (\dot{\delta}, a_x)$.

Position + heading:
$$
\dot{x} = v_x \cos(\psi + \beta), \qquad
\dot{y} = v_x \sin(\psi + \beta), \qquad
\dot{\psi} = \omega
$$

Tire slip angles (front $f$, rear $r$):
$$
\alpha_f = \delta - \arctan2(v_y + l_f \omega,\; v_x), \qquad
\alpha_r = -\arctan2(v_y - l_r \omega,\; v_x)
$$

Linear tire forces with friction $\mu$:
$$
F_{yf} = \mu \cdot F_{zf} \cdot C_{\alpha_f} \cdot \alpha_f, \qquad
F_{yr} = \mu \cdot F_{zr} \cdot C_{\alpha_r} \cdot \alpha_r
$$

Body-frame accelerations:
$$
\dot{v}_y = \frac{1}{m}(F_{yf} \cos\delta + F_{yr}) - v_x \omega, \qquad
\dot{\omega} = \frac{1}{I}\left(l_f F_{yf} \cos\delta - l_r F_{yr}\right)
$$

Integrated with single-step RK4 at `sim_time_step` (= 0.1 s).

---

## 1. High-level architecture

```
        Sensor / state                     Perception                       Control
  ┌──────────────────────┐    ┌──────────────────────────────────┐   ┌──────────────────────────────────┐
  │  /pf/pose/odom ~50Hz │    │  opp_lidar_detector_node (C++)   │   │  mppi_node (Python + JAX)        │
  │  /scan         ~15Hz │───▶│  • cluster non-static LiDAR pts  │   │  ┌────────────────────────────┐  │
  │  /map  (latched)     │    │  • filter by raceline projection │   │  │ pose_callback (cache only) │  │
  └──────────────────────┘    │  • build candidate, score it     │   │  └────────────────────────────┘  │
                              └────────────┬─────────────────────┘   │  ┌────────────────────────────┐  │
                                           │ /opponent/detection_odom│  │ control_timer (40–60 Hz)   │  │
                                           ▼ (~10 Hz)                │  │  ↳ control_step():         │  │
                              ┌──────────────────────────────────┐   │  │     • estimate state       │  │
                              │  opp_predictor_node (C++)        │   │  │     • get reference        │  │
                              │  • KF on (s, v) along raceline   │   │  │     • get opp horizon      │  │
                              │  • forward predict N steps       │   │  │     • apply_auto_*         │  │
                              └────────────┬─────────────────────┘   │  │     • mppi.update()        │  │
                                           │ /opponent/predicted_path│  │     • publish /drive       │  │
                                           ▼ (~20 Hz)                │  └────────────────────────────┘  │
                                                                     │  ┌────────────────────────────┐  │
                                                                     │  │ params timer  (2 Hz)       │  │
                                                                     │  │ stats timer   (0.2 Hz)     │  │
                                                                     │  └────────────────────────────┘  │
                                                                     └────────────────┬─────────────────┘
                                                                                      │ /drive (40–60 Hz)
                                                                                      ▼
                                                                                VESC + steering servo
```

Three independent ROS 2 processes, talking via DDS. The MPPI process runs a **MultiThreadedExecutor** so the lightweight pose-cache callback can update during an in-progress JAX solve.

---

## 2. Timeline — what we changed and why

The order tells a story; the lessons are in the sequence as much as in the individual fixes.

### 2.1 Starting point (`pure-odom` branch baseline)

- Plain MPPI, JAX-based, called inside `pose_callback` synchronously.
- No opponent awareness, no auto-overtake, no guard against bad inputs.
- Worked fine in sim because sim's `/ego_racecar/odom` is steady at high rate and there was no DDS contention.
- **Problem we'd discover later**: the controller's clock was the PF's clock. Anything that slowed PF slowed MPPI proportionally.

### 2.2 First additions (codex draft)

- Pose-delta state estimator (`vy`, `wz` IIR-filtered from PF pose deltas + commanded speed).
- MPPI guards (warm-start saturation reset, finite-output checks, timing-jump reset).
- LiDAR-based opponent detector and a Kalman-filter predictor.
- Opponent cost in MPPI's reward function.

### 2.3 Issue: launching the opponent node killed MPPI

**Symptom**: with opp running, MPPI's `pose_rx` dropped from 50 Hz to 5–10 Hz, control became jerky, the car crashed sometimes. Killing opp didn't always fix it.

**Diagnostic process**:
1. Bag analysis showed PF was bursty (most messages clumped, then long gaps).
2. The IIR vy/wz estimator was 0.65/0.55 prior weights — a single bad sample took ~7 callbacks to wash out.
3. The MPPI solve was happening *inside* the pose callback — so it was yoked to PF cadence.

**Fixes**:
- **Lowered IIR priors to 0.4 / 0.4**, with `dt`-adaptive scaling: when `prev_pose_dt > 0.06 s` the prior drops to 0.35× to flush stale info faster.
- Added **finite-input guards** at every entry to MPPI's compute (odom values, estimated state, reference trajectory, MPPI output).
- Replaced the heavyweight `init_state()` warm-start reset with a cheap `a_opt = 0` zeroing.
- **Soft/hard guard split**: small timing gaps (0.30 s) just log + count, large ones (≥ 1.5 s) wipe the warm-start. This stopped the warm-start from being zeroed on every PF burst.

### 2.4 The big architectural change: timer-driven control loop

The above stabilized things but didn't address the core coupling. **The fix that mattered most**: decouple MPPI from PF cadence entirely.

**Before**:
```python
def pose_callback(self, msg):
    # do estimator, ref traj, MPPI solve, publish /drive — ~16 ms of work
```

**After**:
```python
def pose_callback(self, msg):           # ← microseconds
    self.latest_pose_msg = msg
    self.latest_pose_recv_time = time.time()

def control_timer(self):                 # ← 40 Hz fixed rate
    msg = self.latest_pose_msg
    if pose_age(msg) > 0.20:             # skip if pose is stale
        return
    self.control_step(msg)               # the old work
```

Plus: **MultiThreadedExecutor** with a `ReentrantCallbackGroup` for `pose_callback` and `control_timer`. JAX releases the GIL during GPU work and during host transfers, so the lightweight pose cache update genuinely runs concurrently with the solve.

**Result**: MPPI ticks at its own steady rate even when PF is slow. If PF degrades to 5 Hz under load, MPPI uses the latest available pose; we see it as `pose_age` climbing in the stats line, but `MPPI Hz` stays at the target rate.

### 2.5 Issue: Hz print only flushed at SIGINT

The original code had `print(f"MPPI Hz: {x:.2f}")` inside the callback. Under `ros2 launch`, stdout is block-buffered → the line only appeared when the process shut down.

**Fix**: replaced with a periodic `stats_timer` calling `self.get_logger().info(...)` (rclpy logger is unbuffered) at 0.2 Hz (every 5 s). Plus consolidated all the per-window stats into a single, dense line.

### 2.6 Issue: opponent node was eating CPU even when not detecting anything

**Diagnostic**: scanCallback duration log showed clustering taking 30–80 ms per scan. With `/scan` at 15 Hz that's 50 % of one CPU core. Plus `_pointNearStaticMap_` was reading 169 cells from the static map *per scan point*, on every scan.

**Two algorithmic fixes** in `opponent_lidar_detector_node.cpp`:

1. **O(N²) → O(N) clustering via spatial grid**. Old code: for each point, scanned all N points to find neighbours. New code: hash points into grid cells of size `cluster_tolerance`; only check the 9 neighbouring cells. Functional output identical (same BFS over same neighbour graph), just way fewer pair comparisons.

2. **Per-point inflated-map lookup → precomputed dilation**. Old code recomputed an inflation window around every scan point. New code precomputes `inflated_map_` once per map receipt (or when inflation params change), giving O(1) per point. This was the main contributor to memory-bandwidth contention with JAX's GPU↔host DMA.

**Plus**:
- Moved `refreshLiveParams()` from per-callback (15 Hz × 25 mutex-protected reads) to a 2 Hz timer.
- Added a per-scan duration log (`scanCallback X.XX ms | dyn_pts=N clusters=M`) and rejection-reason tally so you can see *why* clusters are rejected (size / extent / projection / ...).

**Result**: scanCallback went from 30–80 ms → 0.3–1.2 ms. ~50× speedup. MPPI's own solve time stopped degrading when opp launched.

### 2.7 Issue: `get_params` was running 50× per second on the control critical path

**Diagnostic**: even with the timer loop, periodic 200–500 ms spikes. A Claude session running on the Jetson live confirmed MPPI's process had plenty of CPU headroom — so the bug was *inside* MPPI's process. Suspect: GIL.

`get_params()` reads ~50 ROS parameters via the rclpy parameter service, each behind a mutex — pure Python under the GIL. At 40 Hz that's 2000 mutex'd reads per second.

**Fix**: moved `get_params` to a 2 Hz timer in the default callback group. Same pattern as the opp detector. Live tuning latency went from "instant" to "0.5 s" — fine for interactive `ros2 param set`.

### 2.8 Issue: random multi-second freezes

**Symptom**: every 30–60 s, `solve max=500ms` (sometimes `1500 ms`). Not periodic in any obvious way. Happened with and without opp. Almost no JAX log lines around the spike.

**Diagnostic process**:
1. `MPPI_LOG_JAX_COMPILES=1` env var added to enable per-compile logging from JAX.
2. With it on, found `Compiling sample_wall_distance with global shapes [ShapedArray(float32[3,2])]` and later `[float32[5,2]]` — different shapes triggering fresh XLA compiles each time.
3. Cause: `opponent_pass_clearance` was passing a variable-length candidate buffer (size depended on `min(n_check_steps, opp_traj.size, ref_traj.size)`).
4. **Fix #1**: pad to fixed shape `(opponent_auto_check_steps, 2)`. Compile happens once, cached forever.
5. **Fix #2** (added later): pre-warm `sample_wall_distance` at startup with the expected shape so the very first opp encounter doesn't pay the compile cost during driving.

But that didn't explain *all* the spikes. Many remained even without opp running.

**The other half — Python GC**: gen-2 garbage collection sweeps walk the entire heap, holding the GIL for 200–500 ms. JAX's heap is large. With default thresholds, gen-2 fires every ~30 s — exactly the period of the spikes.

- **Wrong fix** (tried first): force `gc.collect()` at 4 Hz to "amortize". Each call still pays the full O(heap) walk cost — made things ~5× worse.
- **Right fix**: `gc.set_threshold(700, 10, 100000)` — gen-0/1 stay automatic (cheap), gen-2 effectively never fires. JAX's heap stabilizes after warmup, so for race-length sessions there's no leak risk.

**Result**: periodic freeze pattern eliminated. Residual ~1 % rare 1-second hiccup remains but it's recoverable (warm-start kept) and doesn't crash the car.

### 2.9 Auto-overtake state machine

The original auto-overtake was stateless — every callback re-evaluated whether to follow / pass-left / pass-right from scratch. Result: at boundary conditions (closing speed dipping near threshold, clearance crossing threshold mid-pass) the mode would flip rapidly → MPPI fought itself.

**Rewrote as a state machine** with:
- States: `idle | follow | pass_left | pass_right`.
- **Enter / exit hysteresis**: enter at strict thresholds (`min_wall_clearance=0.45`, `min_closing_speed=0.4`), exit at looser ones (defaulting to 0.7× / 0.4× of enter).
- **Minimum commit duration** (`min_commit_sec=0.5`): once a side is chosen, lock it in for at least 0.5 s no matter what.
- Out-of-range opp transitions to `idle` (drops opp cost cleanly), not `follow` (which would keep applying it).
- One-shot warning + safe fallback when `wall_sdf` isn't loaded.

**Plus**: fixed a "phantom opponent at origin" bug — `opponent_follow_weight` and `opponent_pass_weight` weren't being gated by `opponent_active` in the cost array, so when no opp was present, the zero-init `opponent_xy_horizon` (which lives at map (0,0)) was treated as a stationary opponent, causing MPPI to swerve toward the origin every lap.

### 2.10 Visualization fix

The optimal trajectory marker in Foxglove was alternating wildly on hardware (but driving was fine). Cause: hardware uses the IIR-estimated `vy`/`wz` in the rollout's initial state, and small noise in those estimates, combined with a multimodal cost landscape, makes the *single winning sample's* trajectory shape flip between callbacks. The actual `/drive` command is the weighted average over all samples plus warm-start EMA, so it's smooth — but the marker shows the winning sample.

**Fix**: roll out `a_opt` a second time from raw twist (no estimator noise) just for the visualization marker. Cost is ~ms, doesn't affect the controller's truth.

---

## 3. Inside the MPPI control loop

Per `control_timer` tick at 40–60 Hz:

```
                      ┌─────────────────────────┐
                      │ Read latest_pose_msg    │
                      │ (atomic ptr swap, no    │
                      │  locking needed)        │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ Check pose_age          │
                      │ if > 0.20s: SKIP        │
                      │ (no /drive republish)   │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ estimate_vehicle_state  │
                      │ (vx, vy, wz, β) via IIR │
                      │ blend of pose-delta +   │
                      │ commanded speed         │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ get_reference_traj      │
                      │ (n_steps points along   │
                      │ raceline at projected   │
                      │ speeds)                 │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ get_opponent_horizon    │
                      │ (latest predicted opp   │
                      │ trajectory, n_steps     │
                      │ × 2)                    │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ apply_auto_opponent_    │
                      │ behavior (state         │
                      │ machine sets mode_id)   │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ mppi.update():          │
                      │  • shift warm-start     │
                      │  • sample N controls    │
                      │  • rollout each via     │
                      │    dynamic_ST           │
                      │  • compute reward       │
                      │  • take reward-weighted │
                      │    average → new a_opt  │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ Guard checks:           │
                      │  • non-finite output    │
                      │  • saturation (warm-    │
                      │    start at limit too   │
                      │    long)                │
                      │  • timing gaps          │
                      └───────────┬─────────────┘
                                  ▼
                      ┌─────────────────────────┐
                      │ Compose /drive:         │
                      │  • steer = a_opt[0,0]   │
                      │    integrated           │
                      │  • speed = blend(MPPI,  │
                      │    raceline profile)    │
                      │  • clamp + rate limit   │
                      └───────────┬─────────────┘
                                  ▼
                              /drive
```

Key design choices:
- **`pose_age` check before solving**: if pose is too old, we *skip* rather than republish a stale /drive. Downstream sees the last command persist; this is safer than publishing a control derived from an out-of-date state.
- **Atomic pointer swap for cache**: `pose_callback` does `self.latest_pose_msg = msg`. CPython attribute assignment is one bytecode → race-free.
- **Speed comes from a blend of MPPI's optimal speed and a raceline-profile feedforward**, controlled by `speed_profile_drive_blend`. 0 = pure MPPI, 1 = pure profile FF. Lets you trust the precomputed speed profile while still letting MPPI shape steering.

---

## 4. Opponent perception pipeline

Two C++ nodes in series.

### 4.1 `opp_lidar_detector_node` — find dynamic objects

Per `/scan` (~15 Hz):

1. **Filter scan points** by range (`min_range`, `max_range`), FOV (`min_base_x`), and reject points that fall on the inflated static map. The inflation mask is precomputed once per map receipt — `wall_inflation_radius` (default 0.3 m) is dilated into the occupancy grid, giving an O(1) per-point lookup.

2. **Cluster the surviving "dynamic" points** with a grid-bucketed BFS (cell size = `cluster_tolerance`, default 0.18 m). Output: list of clusters, each a vector of point indices.

3. **For each cluster, build a candidate** and check it against gates:
   - `min_cluster_points` ≤ count ≤ `max_cluster_points` (3 to 200, by default).
   - Centroid `base_x` ≥ `min_base_x` (-0.5 m — opp must be roughly in front).
   - Centroid distance to nearest raceline waypoint ≤ `max_raceline_projection_dist` (0.6 m hardware). **This is the most important tuning gate.**
   - Cluster extent in raceline-tangent and -normal frames within `[min_cluster_extent, max_cluster_extent]` (0.05 to 0.90 m). The 0.90 m upper bound covers diagonal corner views of the F1TENTH car (diagonal ≈ 0.66 m).
   - "Center correction" — shifts the centroid toward the visible edge to estimate the opp's true center, capped by `max_center_correction`.
   - Continuity gate: if there was a recent detection, the new candidate must be within `max_candidate_jump` and `max_detection_speed`.

4. **Score surviving candidates** by `projection_weight × proj_distance + range_weight × range + continuity_weight × jump`. Pick the lowest score. Publish `/opponent/detection_odom` with the candidate's center as a pose.

5. **Throttled diagnostic line every 2 s**:
   ```
   scanCallback 0.5 ms | dyn_pts=21 clusters=3 | accepted=3
   reject{size=0, base_x=0, proj=0, extent=0, proj_post=0, speed=0, jump=0}
   best_proj_d=0.08 (thresh=0.60)
   ```
   Tells you exactly which gate is rejecting — invaluable for tuning.

### 4.2 `opp_predictor_node` — track and predict

Per `/opponent/detection_odom` (~10 Hz):
1. Project the detected pose onto the raceline → `s` (arc length) and lateral offset.
2. Update a **1-D Kalman filter** in `(s, v)` space: state is `(arc-length, speed-along-raceline)`, dynamics are *constant velocity along the raceline*, measurements are the projected `s` and a derived progress speed (Δs / Δt). If the per-tick progress-speed measurement is invalid (NaN / negative / above `max_detection_speed`), it can be substituted with the raceline's reference speed at the current `s` via `use_profile_speed_fallback`, so the filter still updates instead of stalling.

Per timer (20 Hz):
1. Predict the KF forward to `now()`.
2. **Generate the published horizon — shape-preserving propagation**, NOT pure constant-velocity and NOT absolute blend toward raceline pace. The KF velocity is the anchor at step 0; the raceline contributes only its *deltas*. For `prediction_steps` future steps (default 5, each `prediction_dt = 0.1` s apart):
   - Step in arc-length: `next_s = pred_s + pred_v * dt`.
   - Look up the raceline reference speed at the next arc length: `next_ref_v = interpolateTrack(next_s).speed`.
   - Compute how the raceline's reference speed evolves over this step: `delta_ref = next_ref_v - prev_ref_v`.
   - Apply β fraction of that delta to the predicted speed: `pred_v = max(0, pred_v + β * delta_ref)`. β = `profile_speed_blend` (0.7) when fresh, `out_of_sight_profile_speed_blend` (1.0) when stale.
   - Place the horizon waypoint at the raceline XY corresponding to `next_s`.

   This is the mechanism that lets the prediction respect raceline-driven braking and acceleration **while preserving the opponent's measured pace offset**. An opponent measured at 3 m/s where the raceline says 5 m/s gets predicted to brake into the next turn (because the raceline reference speed drops there) but stays roughly 2 m/s slower than the raceline throughout — far more representative than the alternative of "converge to raceline pace within 3 steps". The KF stays simple (clean noise model); the "opponents follow the raceline's *shape* (not its absolute pace)" prior lives in the propagation step.
3. Smooth the lateral offset (the opp may be off-line) with a separate IIR alpha filter, decaying to zero over `lateral_offset_decay_time` so far-future predictions tend back toward the raceline center.
4. Publish:
   - `/opponent/odom` — current best estimate as Odometry.
   - `/opponent/predicted_path` — the N-step horizon as a `nav_msgs/Path`.
   - markers + debug.

If no detection has arrived for `stale_timeout` (0.5 s), publishes an *empty* path so MPPI knows opp is gone.

### 4.3 What MPPI does with the predicted path

`opponent_path_callback` in MPPI fills `self.opponent_xy_horizon` (shape `(n_steps, 2)`) from the path. The cost function uses it three ways:

- **Keep-out (`opponent_cost_weight`)**: penalizes any rollout point closer than `opponent_cost_radius` to the opp's predicted position at the same horizon step.
- **Follow (`opponent_follow_weight`)** when in follow mode: penalizes rollouts that close the longitudinal gap below `opponent_follow_distance`, but only if ego is in roughly the same lane (within `opponent_same_lane_width`).
- **Pass (`opponent_pass_weight`)** when in pass-left/right mode: penalizes rollouts that don't achieve the requested lateral offset (`opponent_pass_lateral_offset`) on the chosen side, within a longitudinal window (`opponent_pass_longitudinal_window`).

All three are gated by `opponent_active` (which becomes false when the path is empty/stale) — so when no opp is present, none of these costs fire.

---

## 5. Auto-overtake decision algorithm

This is the brain that decides "follow", "pass left", "pass right", or "ignore". It runs inside MPPI's `apply_auto_opponent_behavior` per control tick.

**Only runs when `opponent_behavior_mode: auto` in the YAML.** Other modes (`follow`, `clear`, `pass_left`, `pass_right`) bypass it entirely.

### 5.1 The inputs every tick

- **Geometry**: ego XY + yaw, opp XY (from horizon[0]), reference yaw (from raceline at ego).
- `ahead_distance` = projection of (opp − ego) onto the reference tangent. Positive means opp is ahead of ego along the raceline.
- `closing_speed` = ego's desired speed (max of measured and reference) − opp's speed (estimated from horizon[1] − horizon[0]).
- `left_clearance` and `right_clearance` = the wall-distance value at a probe point offset `pass_offset` to the left/right of opp (sampled from the precomputed wall SDF).

### 5.2 The state machine

```
                      ┌──────────────────────────────────┐
                      │ if mode != 'auto': do nothing.   │
                      └──────────────────┬───────────────┘
                                         ▼
       ┌──────────────────────────────────────────────────┐
       │ if NOT opponent_active OR opp out of range:      │
       │     state = idle; mode_id = 1.0 (clear)          │
       │     → no opponent costs fire.                    │
       └──────────────────────────────┬───────────────────┘
                                      ▼
       ┌────────────────────────────────────────────────────┐
       │ if state ∈ {pass_left, pass_right}:                │
       │   if time_in_state < min_commit_sec (0.5):         │
       │     stay locked in.                                │
       │   else if EXIT thresholds violated (side_clear     │
       │           < exit_clear OR closing < exit_closing): │
       │     log "aborting", → state = follow.              │
       │   else: stay in current pass.                      │
       └──────────────────────────────┬─────────────────────┘
                                      ▼  (only reached if not committed)
       ┌────────────────────────────────────────────────────┐
       │ ENGAGEMENT:                                        │
       │   if closing < enter_closing (0.4 m/s):            │
       │     state = follow; mode_id = 0.0                  │
       │   elif neither side has enter_clear (0.45 m):      │
       │     state = follow; mode_id = 0.0                  │
       │   else: pick side (better clearance + side_margin  │
       │     hysteresis), state = pass_<side>.              │
       └────────────────────────────────────────────────────┘
```

**Mode IDs** consumed by the JAX cost function:
- `0.0` = follow  (follow + keep-out costs active)
- `1.0` = clear   (no opp cost active — used when irrelevant)
- `2.0` = pass left
- `3.0` = pass right

The hysteresis is the key idea: enter is strict so we don't engage spuriously; exit is loose so a transient dip in clearance/closing during a pass doesn't abort it. Plus `min_commit_sec` ensures we don't oscillate at boundary conditions.

---

## 6. Tuning guide — knob → symptom

### 6.1 The MPPI itself

| param | what it does | when to raise | when to lower |
|---|---|---|---|
| `n_samples` | number of sampled control sequences per solve | smoother behaviour, better global optimum | reduce when solve_max is hitting the period |
| `n_steps` | rollout horizon length | more lookahead → better turn entry | reduce if MPPI is too slow to commit to corrections |
| `temperature` | reward-weighting sharpness (lower = winner-take-all) | higher → smoother averaged action; raises if traj_opt viz is jumpy | lower if you want more decisive single-sample winners |
| `xy_reward_weight` | path tracking gain | tighter line | car becomes too eager to swerve back |
| `velocity_reward_weight` | speed tracking gain | hits target speed faster | smoother accel |
| `yaw_reward_weight` | heading tracking gain | better turn entry | reduce if it fights steering |
| `friction` | tire/surface friction model | higher → believes it has more grip → tighter lines | lower if car oversteers / understeers |
| `control_loop_hz` | controller tick rate | more responsive (until period < solve_p99) | more headroom |
| `control_pose_stale_sec` | skip threshold | larger = tolerate slower PF | tighten if you want fresher pose |

### 6.2 Wall cost

| param | what it does |
|---|---|
| `wall_cost_enabled` | master switch |
| `wall_cost_weight` | how strongly walls repel |
| `wall_cost_margin` | distance from a wall at which cost starts (m) |
| `wall_cost_power` | exponent on `(margin − dist)` |
| `wall_cost_map_yaml` | map used to build the SDF (injected by launch) |

Increase `wall_cost_weight` if MPPI hugs walls too closely. Increase `wall_cost_margin` if you want a bigger safety bubble (but it will affect the racing line).

### 6.3 Opponent detector

| param | what it does | symptom if too tight | symptom if too loose |
|---|---|---|---|
| `max_raceline_projection_dist` | max cluster-to-raceline distance to be opp | misses opp when off-line / passing | accepts walls/noise as opp |
| `min_cluster_points` | min LiDAR hits per cluster | misses far/oblique opps | accepts noise blobs |
| `min_cluster_extent` / `max_cluster_extent` | size band of cluster bounding box | misses corner views (>0.65) or partial side views (<0.08) | accepts walls / multi-object groups |
| `cluster_tolerance` | grid cell size for connecting points | fragments single car into pieces | merges separate objects |
| `wall_inflation_radius` | dilation of static map (m) | opp near wall gets filtered as wall | fails to filter scan noise from walls |
| `min_base_x` | how far behind ego to look (m) | misses opp slightly behind | accepts opp's reflection / behind-view noise |

**Diagnostic**: the throttled `scanCallback` log tells you which gate is rejecting. Use it to tune.

### 6.4 Opponent predictor

| param | what it does |
|---|---|
| `prediction_steps` | how many future steps to predict (matches MPPI's `n_steps`) |
| `prediction_dt` | step duration (matches MPPI's `sim_time_step`) |
| `kf_process_var_s` / `kf_process_var_v` | KF process noise — higher = more responsive, less smoothed |
| `kf_measurement_var_s` / `kf_measurement_var_v` | KF measurement noise — higher = trust the model more, the measurement less |
| `profile_speed_blend` (β, fresh detections) | strength of the **shape-preserving** raceline coupling. KF speed is the anchor; β scales how much of the raceline's per-step velocity delta is added. **0 = pure constant-velocity from KF (no raceline shape), 1 = preserve KF–raceline offset exactly through the horizon (full raceline shape).** Default 0.7 = 70 % of the raceline's per-step change. Raise toward 1 if opponents brake/accelerate strongly with the raceline; lower toward 0 if their speed is uncorrelated with the line. **Note**: this is *not* a blend toward absolute raceline pace — see §0b math for the exact formula. |
| `out_of_sight_profile_speed_blend` (β, stale detections) | same shape-preserving formula but with this β when detection has gone stale. Default 1.0 = preserve last-known offset and follow raceline shape exactly while we wait for a new detection. |
| `use_profile_speed_fallback` | if true, substitute raceline reference speed for an invalid measured progress speed in the KF update (instead of skipping the update). Useful when detection is jittery. |
| `stationary_speed_threshold` | KF velocity below this gets clamped to 0 (treats opp as parked). |
| `hold_stationary_when_stale` | if true, keep predicting opp as stationary even after going stale (rather than letting the raceline-pace blend drag the prediction forward). |
| `lateral_offset_alpha` | IIR weight on lateral offset estimate |
| `lateral_offset_decay_time` | time constant for lateral offset decaying back to raceline center over the horizon |
| `stale_timeout` | sec without detection before publishing empty path |
| `max_progress_speed` / `max_progress_accel` | clamps on the measured progress speed/accel before it's fed to the KF — rejects projection-noise spikes |
| `max_stale_prediction_time` | how far past the last measurement we'll continue to predict |

### 6.5 Opponent cost & auto-overtake

| param | what it does | tune up if | tune down if |
|---|---|---|---|
| `opponent_cost_weight` | strength of the keep-out repulsion | car drifts into opp | car panics around opp |
| `opponent_cost_radius` | radius of the keep-out bubble | gives opp more space | tighter passes |
| `opponent_follow_weight` | strength of follow-distance penalty | car runs into opp's bumper | car slows down too eagerly |
| `opponent_follow_distance` | desired gap behind opp (m) | want bigger trailing gap | tighter follow |
| `opponent_pass_weight` | strength of pass-side bias | pass commits more strongly to a side | pass is too aggressive |
| `opponent_pass_lateral_offset` | desired lateral offset during pass | bigger gap to opp during pass | tighter pass (but more risk) |
| `opponent_auto_max_ahead_distance` | only consider passing opps within this distance (m) | engage on more distant opps | wait until opp is close |
| `opponent_auto_min_wall_clearance` | required wall clearance to *enter* a pass (m) | safer (refuses tighter passes) | more aggressive (passes through narrower gaps) |
| `opponent_auto_min_closing_speed` | required closing speed to enter (m/s) | pass only when clearly catching | engage even on near-equal speed |
| `opponent_auto_min_commit_sec` | hold a chosen side for at least this long (s) | smoother / less flippy | more reactive |
| `opponent_auto_exit_wall_clearance` | clearance below which we abort mid-pass (m, 0=auto) | aborts more aggressively | commits harder to passes |
| `opponent_auto_exit_closing_speed` | closing below which we abort mid-pass (m/s, 0=auto) | aborts when opp pulls away | commits harder |

**Tuning workflow for auto-overtake**:
1. Start with `mode: follow` and confirm follow distance is good.
2. Switch to `mode: pass_left` and `pass_right` manually to tune the pass-cost bias.
3. Switch to `mode: auto`. Watch:
   - `/mppi/debug/opponent_auto_mode_id` (0/1/2/3)
   - `/mppi/debug/opponent_auto_left_clearance`, `right_clearance`
   - `/mppi/debug/opponent_auto_closing_speed`
   - `/mppi/debug/opponent_auto_pass_allowed`
4. If mode flickers rapidly between values: raise `min_commit_sec`.
5. If passes never engage when you expect them to: check `pass_allowed=0` topic, find which gate is blocking via the clearance/closing values.
6. If passes engage too early / for too-distant opps: lower `max_ahead_distance`.

### 6.6 Optional dynamic costs (currently mostly off)

- `slip_cost_*` — penalizes large `|β|` (sideslip angle). Useful if car is oversteering at the limit.
- `latacc_cost_*` — penalizes large `|v · ω|` (lateral acceleration). Useful as a soft tire-grip ceiling.
- `steer_sat_cost_*` — penalizes near-saturation steering. Useful if the optimizer is constantly maxing out the wheel.

These are off by default. If you turn them on, set the safety threshold (`*_safe`) realistically — e.g. `slip_cost_beta_safe: 0.15` (≈ 8.6°), not the default `0.5` (≈ 28°, which is post-spin).

---

## 7. Stats line glossary

The MPPI process prints this every 5 seconds:

```
MPPI 50.0Hz | pose_rx 248Hz | drive_tx 50.0Hz | ctrl_ticks 50.0Hz
(skip stale=0 no_pose=0) | solve mean=8ms p99=18ms max=22ms
| pose_age=2ms | guard+=0 (soft+=0) | opp_rx 20Hz
```

| field | meaning | healthy value | bad sign |
|---|---|---|---|
| `MPPI Hz` | rate of /drive publication (= drive_tx) | matches `control_loop_hz` | drops when solve > period or pose stale |
| `pose_rx Hz` | pose_callback invocation rate | matches PF publish rate (~50 Hz) | < 30 = PF degrading or executor starved |
| `drive_tx Hz` | /drive send rate | = MPPI Hz | should never differ |
| `ctrl_ticks Hz` | timer firing rate | = `control_loop_hz` | < target = executor blocked |
| `skip stale=N` | timer ticks skipped because pose was older than `control_pose_stale_sec` | 0 | > 0 = PF lagging behind |
| `no_pose=N` | timer fired before any pose ever arrived | 0 after first second | persistent = PF down |
| `solve mean / p99 / max` | JAX solve time stats over the 5s window (ms) | mean < 25, p99 < 50 | spikes > 100 ms = recompile or contention |
| `pose_age` | how old the pose was when last consumed (ms) | < 30 | > 100 = stale plan |
| `guard+= N` | HARD guards (warm-start wiped) in the window | 0 | > 0 = real stalls (>1.5s gap or non-finite) |
| `soft+= N` | SOFT guards (timing gap noted, warm-start kept) | 0–1 | climbing = PF jitter or executor contention |
| `opp_rx Hz` | opponent path messages received | 0 if no opp detected | otherwise matches predictor's publish rate (~20 Hz) |

**Two relationships matter**:
- `ctrl_ticks > MPPI Hz` → timer is fine, control_step is being skipped (pose stale). PF problem.
- `solve_mean ≈ 1 / MPPI Hz` → MPPI is solve-bound. Reduce `n_samples` or `n_steps`.

---

## 8. Why we made the unusual choices we did

Things you might be asked about during the presentation.

- **Why a timer instead of an event-driven loop?** Decouples controller cadence from PF cadence so PF degradation doesn't slow control.
- **Why MultiThreadedExecutor?** So pose_callback (cache-only) can run on a different thread than the JAX solve. JAX releases the GIL during GPU work.
- **Why disable Python gen-2 GC?** Each gen-2 sweep walks the JAX heap (huge), 200–500 ms. Periodic enough to be noticeable. JAX heap stabilizes after warmup so we don't need to free it during a race.
- **Why precompute the inflated map instead of inflating on the fly?** Per-scan-point inflation lookup was 169 random memory accesses per point × 1080 points = 180k random accesses per scan, contending with JAX's GPU↔host DMA for memory bandwidth.
- **Why grid-bucketed clustering instead of sklearn DBSCAN or anything else?** No external dependencies, ~50 lines of C++, 50× faster than the original O(N²).
- **Why state machine for auto-overtake instead of stateless re-evaluation?** Hysteresis. Enter strict, exit loose, commit for a minimum time. Stops boundary flicker.
- **Why a fixed-shape candidate buffer in `opponent_pass_clearance`?** JAX recompiles when input shape changes. Fixing the shape at `(opponent_auto_check_steps, 2)` means the function compiles once and is cached forever.
- **Why a soft/hard guard split?** PF stamps come in bursts under driving load. The original "any gap > 0.12s wipes warm-start" zeroed the warm-start ~once per second on the Jetson, breaking control continuity. Soft path keeps the warm-start (still mostly valid for a 0.3-1.5s gap), hard path wipes it (only for true stalls).
- **Why pre-warm `sample_wall_distance`?** First-time XLA compile is ~75–150 ms. Without pre-warm, this happens the first time the car sees an opponent — exactly when continuity matters most.

---

## 9. Failure modes & how the system recovers

| failure | detection | recovery |
|---|---|---|
| Non-finite odom input | `np.isfinite(pose_values).all()` check | clear persistent state, reset estimator, return early |
| Non-finite estimated state | same check on `state_c_0` | same |
| Non-finite reference trajectory | check on `reference_traj` | same |
| Non-finite MPPI output | check on `a_opt_cpu` and `traj_opt` | same; falls back to startup speed after K consecutive failures |
| Warm-start saturation (a_opt at limit too long) | `aopt_max_abs >= threshold` for K callbacks | clear warm-start (but not state estimator) |
| Multi-second wall callback gap | `wall_dt > mppi_guard_hard_gap` (1.5s default) | clear warm-start + reset estimator |
| Non-monotonic pose stamps | `stamp_dt <= 0` | clear warm-start |
| Brief pose staleness | `pose_age > control_pose_stale_sec` (0.20s) | skip the solve, no /drive update; downstream holds last command |
| PF starvation under load | `pose_rx Hz` drops in stats line | controller continues at its own rate using the latest cached pose |

Every guard event is counted (`guard+=` and `soft+=` in the stats line) and logged with a reason. After 100 events, the user can grep for the rate in the launch log.

---

## 10. Configuration files map

```
mppi_bringup/
├── config/
│   ├── params_lev.yaml                  ← hardware MPPI params (Levine map)
│   ├── params_realev_overtake.yaml      ← hardware MPPI + opponent
│   ├── params_sim_lev.yaml              ← sim MPPI
│   └── params_sim_lev_overtake.yaml     ← sim MPPI + opponent
├── launch/
│   ├── lev.launch.py                    ← hardware MPPI alone
│   ├── lev_overtake.launch.py           ← hardware MPPI + opp
│   ├── sim.launch.py
│   └── sim_overtake.launch.py
└── waypoints/
    └── lev_testing/
        ├── racetrack_lev_real1.csv      ← raceline used on hardware
        └── ...

opponent_predictor/
├── config/
│   ├── hardware/
│   │   ├── lidar_detector.yaml          ← detector params (real)
│   │   └── params.yaml                  ← predictor params (real)
│   └── sim/
│       ├── lidar_detector.yaml          ← detector params (sim)
│       └── params.yaml                  ← predictor params (sim)
└── launch/
    ├── real_opponent_predictor.launch.py
    └── sim_opponent_predictor.launch.py

mppi_example/
└── mppi_example/
    ├── mppi_node.py                     ← all of the rclpy + state machine
    ├── mppi_tracking.py                 ← MPPI sampling + reward weighting
    ├── infer_env.py                     ← cost function + reference traj
    └── dynamics_models/
        └── dynamics_models_jax.py       ← single-track dynamics
```

---

## 11. Quick-start for tuning

If something looks wrong, in order:

1. **Look at the stats line**. `MPPI Hz`, `solve max`, `pose_age`, `guard+=`, `opp_rx`. The fault is almost always visible there.
2. **Check the throttled detector log** (`scanCallback ... reject{...}`). Tells you if perception is the limit.
3. **Check the auto-overtake debug topics** if pass behaviour looks wrong:
   ```
   /mppi/debug/opponent_auto_mode_id
   /mppi/debug/opponent_auto_left_clearance
   /mppi/debug/opponent_auto_right_clearance
   /mppi/debug/opponent_auto_closing_speed
   /mppi/debug/opponent_auto_pass_allowed
   ```
4. **For perf debugging**, set `MPPI_LOG_JAX_COMPILES=1` env var to log every JAX compile. After warmup there should be none. If there are during steady state, there's a shape-variation bug to fix.
5. **For tuning live**, `ros2 param set /lmppi_node <name> <value>` — most params refresh at 2 Hz.
