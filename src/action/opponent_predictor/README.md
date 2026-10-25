# Opponent Predictor

This package predicts where an opponent car is now and where it is likely to be over a short horizon. It is designed to feed future MPPI opponent-avoidance costs, while staying simple enough to tune and explain for the final project.

The key idea is to predict in **track progress coordinates** instead of raw Cartesian coordinates. A linear $x,y$ prediction drives straight through corners, while a raceline-progress prediction naturally follows the track curvature.

The full pipeline is:

$$
\text{LiDAR scan} + \text{ego pose} + \text{static map}
\rightarrow
\text{opponent detection odom}
\rightarrow
\text{raceline-progress predictor}
\rightarrow
\text{future opponent path}
$$

In ROS topics:

$$
/scan,\ /ego\_racecar/odom,\ /map
\rightarrow
/opponent/detection\_odom
\rightarrow
/opponent/odom,\ /opponent/predicted\_path
$$

## LiDAR Opponent Detection

The first node, `opponent_lidar_detector_node`, estimates the opponent pose from LiDAR. It does not try to predict the opponent. It only tries to answer:

> Is there a plausible opponent-sized dynamic object visible right now, and if so, where is its approximate center?

The detector intentionally fails silent when uncertain. If it does not publish a fresh detection, the predictor keeps using its stale raceline-based prediction for a short time.

### Scan Point Transform

Each valid LiDAR range point is first converted into the laser frame:

$$
\mathbf{p}_{L}
=
\begin{bmatrix}
r\cos\theta \\
r\sin\theta
\end{bmatrix}
$$

The configurable laser offset maps this point into the ego base frame:

$$
\mathbf{p}_{B}
=
\begin{bmatrix}
x_L \\
y_L
\end{bmatrix}
+
\mathbf{R}(\psi_L)
\mathbf{p}_{L}
$$

where $(x_L, y_L, \psi_L)$ are `laser_x_offset`, `laser_y_offset`, and `laser_yaw_offset`.

The ego odometry then maps the point into the global map frame:

$$
\mathbf{p}_{M}
=
\begin{bmatrix}
x_e \\
y_e
\end{bmatrix}
+
\mathbf{R}(\psi_e)
\mathbf{p}_{B}
$$

Only points within the configured range limits are kept:

$$
r_{\min} \le r \le r_{\max}
$$

and:

$$
r_{\text{detect,min}} \le \|\mathbf{p}_{B}\| \le r_{\text{detect,max}}
$$

### Static Map Filtering

The detector removes points that fall on the static map. A scan point is rejected if its map cell or nearby inflated cells are occupied:

$$
\text{occupied}(\mathbf{p}_{M}) =
\max_{\mathbf{q} \in \mathcal{N}(\mathbf{p}_{M}, r_{\text{wall}})}
\text{map}(\mathbf{q})
\ge
\text{occupied\_threshold}
$$

where $r_{\text{wall}}$ is `wall_inflation_radius`.

This is the main separation between:

- static walls and map structure;
- dynamic objects that are visible in LiDAR but not part of the map.

### Clustering

Remaining dynamic points are clustered with Euclidean connectivity. Two points belong to the same cluster if:

$$
\|\mathbf{p}_i - \mathbf{p}_j\| \le r_{\text{cluster}}
$$

where $r_{\text{cluster}}$ is `cluster_tolerance`.

Clusters are rejected if they are too small, too large, or have implausible physical extent:

$$
N_{\min} \le N \le N_{\max}
$$

$$
e_{\min} \le \max(e_t, e_n) \le e_{\max}
$$

where $e_t$ and $e_n$ are cluster extents along the raceline tangent and normal directions.

### Raceline-Based Candidate Gating

For each cluster, the visible centroid is:

$$
\mathbf{c}_{\text{vis}}
=
\frac{1}{N}
\sum_{i=1}^{N}
\mathbf{p}_i
$$

The centroid is projected onto the raceline. The detector rejects clusters whose projection distance is too large:

$$
d_{\text{proj}}(\mathbf{c}_{\text{vis}})
\le
d_{\text{proj,max}}
$$

This ended up being the most important false-positive filter. In our debug bags, true opponent detections were usually very close to the raceline, while wall/noise detections tended to sit much farther away.

### Dimension-Aware Center Correction

A raw LiDAR cluster centroid is usually not the center of the opponent car. Depending on geometry, ego may see:

- the back face of the car;
- the side of the car;
- a diagonal slice while the car turns.

So the detector estimates a center correction using the known approximate car dimensions.

Let $\mathbf{t}$ be the raceline tangent and $\mathbf{n}$ the raceline normal at the projected cluster location:

$$
\mathbf{t}
=
\begin{bmatrix}
\cos\psi_r \\
\sin\psi_r
\end{bmatrix}
$$

$$
\mathbf{n}
=
\begin{bmatrix}
-\sin\psi_r \\
\cos\psi_r
\end{bmatrix}
$$

The unit vector from ego to the visible cluster is:

$$
\mathbf{u}
=
\frac{\mathbf{c}_{\text{vis}}-\mathbf{p}_{ego}}
{\|\mathbf{c}_{\text{vis}}-\mathbf{p}_{ego}\|}
$$

The observed extents along raceline tangent and normal are:

$$
e_t =
\max_i ((\mathbf{p}_i-\mathbf{c}_{\text{vis}})^T\mathbf{t})
-
\min_i ((\mathbf{p}_i-\mathbf{c}_{\text{vis}})^T\mathbf{t})
$$

$$
e_n =
\max_i ((\mathbf{p}_i-\mathbf{c}_{\text{vis}})^T\mathbf{n})
-
\min_i ((\mathbf{p}_i-\mathbf{c}_{\text{vis}})^T\mathbf{n})
$$

The missing half-depths are approximated as:

$$
m_t =
\max\left(0,\ \frac{L_{\text{car}}}{2} - \frac{e_t}{2}\right)
$$

$$
m_n =
\max\left(0,\ \frac{W_{\text{car}}}{2} - \frac{e_n}{2}\right)
$$

The center correction is:

$$
\Delta \mathbf{c}
=
\mathbf{t}(\mathbf{u}^T\mathbf{t})m_t
+
\mathbf{n}(\mathbf{u}^T\mathbf{n})m_n
$$

and is capped by `max_center_correction`:

$$
\|\Delta \mathbf{c}\|
\le
\Delta c_{\max}
$$

The estimated opponent center is:

$$
\mathbf{c}_{\text{center}}
=
\mathbf{c}_{\text{vis}}
+
\Delta \mathbf{c}
$$

The detector publishes this as `/opponent/detection_odom`. The yaw is taken from the raceline yaw, not from noisy cluster orientation.

### Candidate Scoring

Among valid clusters, the detector picks the lowest-cost candidate:

$$
J_{\text{cluster}}
=
w_d d_{\text{proj}}
+
w_r r
+
w_c \|\mathbf{c}_{\text{center}}-\mathbf{c}_{\text{last}}\|
$$

where:

- $d_{\text{proj}}$ is distance from raceline;
- $r$ is range from ego;
- the continuity term is only active when a recent previous detection exists.

Candidate jumps are rejected if the implied progress speed is too large:

$$
\left|
\frac{s_t - s_{t-1}}{\Delta t}
\right|
>
v_{\text{detect,max}}
$$

This prevents cluster switching from creating huge fake opponent speeds.

## State And Measurements

The filter state is:

$$
\mathbf{x} =
\begin{bmatrix}
s \\
v
\end{bmatrix}
$$

where:

$$
s = \text{distance/progress along the raceline}
$$

$$
v = \dot{s}
$$

The opponent odometry gives a Cartesian pose:

$$
\mathbf{p} =
\begin{bmatrix}
x \\
y
\end{bmatrix}
$$

That pose is projected onto the raceline to obtain:

$$
s_{\text{meas}}, \quad e_y, \quad d_{\text{proj}}
$$

where $e_y$ is the signed lateral offset from the raceline and $d_{\text{proj}}$ is the projection distance.

## Raceline Projection

For each raceline segment with endpoints $\mathbf{p}_i$ and $\mathbf{p}_{i+1}$, compute:

$$
\alpha =
\text{clip}
\left(
\frac{(\mathbf{p} - \mathbf{p}_i)^T(\mathbf{p}_{i+1}-\mathbf{p}_i)}
{\|\mathbf{p}_{i+1}-\mathbf{p}_i\|^2},
0,
1
\right)
$$

The projected point is:

$$
\mathbf{p}_{\text{proj}} =
\mathbf{p}_i + \alpha(\mathbf{p}_{i+1}-\mathbf{p}_i)
$$

The measured track progress is:

$$
s_{\text{meas}} =
s_i + \alpha(s_{i+1}-s_i)
$$

The signed lateral offset is:

$$
e_y =
(\mathbf{p} - \mathbf{p}_{\text{proj}})^T\mathbf{n}_i
$$

where $\mathbf{n}_i$ is the raceline normal.

Because the track is closed, $s$ is wrapped to:

$$
s \in [0, L)
$$

where $L$ is the total track length. When computing differences between two progress values, the predictor unwraps $s$ so crossing the start line does not look like a large jump.

## Speed Measurement

The primary speed measurement is progress speed:

$$
v_{\text{meas}} =
\frac{s_t - s_{t-1}}{\Delta t}
$$

This is preferred over raw odometry twist because the predictor cares about how quickly the opponent is moving along the track.

The measurement is rejected or clamped if it is physically unreasonable:

$$
0 \le v_{\text{meas}} \le v_{\max}
$$

and speed changes can be limited with:

$$
|v_{\text{meas},t} - v_{\text{meas},t-1}|
\le
a_{\max}\Delta t
$$

If progress speed is unavailable or invalid, the node falls back to twist magnitude:

$$
v_{\text{twist}} =
\sqrt{v_x^2 + v_y^2 + v_z^2}
$$

If both are poor, it can fall back to the raceline profile speed at the current progress.

## Kalman Filter

The predictor uses a constant-velocity Kalman filter in raceline progress:

$$
\mathbf{x}_{k+1}
=
\mathbf{A}\mathbf{x}_k
$$

$$
\mathbf{A}
=
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
$$

Equivalently:

$$
s_{k+1} = s_k + v_k\Delta t
$$

$$
v_{k+1} = v_k
$$

The measurement is:

$$
\mathbf{z}_k =
\begin{bmatrix}
s_{\text{meas}} \\
v_{\text{meas}}
\end{bmatrix}
$$

with measurement model:

$$
\mathbf{z}_k =
\mathbf{H}\mathbf{x}_k
$$

$$
\mathbf{H}
=
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix}
$$

This is a standard linear Kalman filter, not an EKF. A constant-acceleration model with state $[s, v, a]^T$ would also still be linear:

$$
\begin{bmatrix}
s_{k+1} \\
v_{k+1} \\
a_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
1 & \Delta t & \frac{1}{2}\Delta t^2 \\
0 & 1 & \Delta t \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
s_k \\
v_k \\
a_k
\end{bmatrix}
$$

We currently prefer the constant-velocity model because acceleration estimates would require differentiating noisy projected speed measurements. The raceline speed profile already provides a cleaner way to bias future prediction toward slowing for turns and speeding up on straights.

## Position Snapping

For visualization and obstacle-cost anchoring, the current predicted pose should not lag behind the measured opponent pose. After the Kalman update, the progress estimate is optionally snapped toward the latest measurement:

$$
s_{\text{hat}}
\leftarrow
(1-\alpha_{\text{snap}})s_{\text{hat}}
+
\alpha_{\text{snap}}s_{\text{meas}}
$$

With:

$$
\alpha_{\text{snap}} = 1
$$

the current marker is anchored to the newest odometry projection, while velocity remains filtered.

## Future Prediction

The prediction horizon starts from the filtered current state:

$$
s_0 = s_{\text{hat}}
$$

$$
v_0 = v_{\text{hat}}
$$

At each future step, the position is obtained by interpolating the raceline:

$$
\mathbf{p}_k = \mathbf{r}(s_k)
$$

The future speed is blended toward the raceline speed profile:

$$
v_{k+1}
=
(1-\beta)v_k
+
\beta v_{\text{profile}}(s_k)
$$

Then progress advances:

$$
s_{k+1}
=
s_k + v_{k+1}\Delta t
$$

When the opponent is visible, $\beta$ is `profile_speed_blend`. When the opponent is stale or temporarily out of sight, $\beta$ becomes `out_of_sight_profile_speed_blend`.

This means:

- with $\beta = 0$, prediction follows filtered opponent speed;
- with $\beta = 1$, prediction follows the raceline speed profile;
- with small $\beta$, prediction mostly follows the measured opponent but slowly returns toward expected track behavior.

## Lateral Offset

The opponent is not forced exactly onto the raceline. The node filters the measured lateral offset:

$$
e_{y,\text{hat}}
\leftarrow
(1-\alpha_y)e_{y,\text{hat}}
+
\alpha_y e_{y,\text{meas}}
$$

Future lateral offset decays back toward the raceline:

$$
e_y(k)
=
e_{y,\text{hat}}
\exp
\left(
-\frac{k\Delta t}{\tau_y}
\right)
$$

The final predicted position is:

$$
\mathbf{p}^{\text{pred}}_k
=
\mathbf{r}(s_k) + e_y(k)\mathbf{n}(s_k)
$$

This lets the prediction start from the opponent's measured side of the track, while assuming it gradually returns toward the reference.

## Main Parameters

### Predictor Parameters

`waypoint_path`
: Raceline CSV used for projection and prediction. This must match the current map and track.

`reverse_waypoints`
: Set `true` when racing the track in the opposite direction to the centerline CSV. The node flips the loaded waypoints at load time (point order reversed, progress `s` recomputed as cumulative chord length, headings rotated by $\pi$) so opponent progress runs *with* travel. Without it, the opponent moves toward decreasing `s`: progress speed is measured negative and rejected, and the horizon propagates the wrong way around the loop. Keep this in sync with the detector's `reverse_waypoints` so both nodes share one progress frame.

`frame_id`
: Output frame, usually `map`.

`pose_source`
: `odom_pose` trusts map-frame odometry directly. `tf` uses the opponent child-frame transform.

`prediction_steps`
: Number of future points to publish.

`prediction_dt`
: Time spacing between predicted points.

`profile_speed_blend`
: Blend toward raceline speed profile while opponent odometry is fresh.

`out_of_sight_profile_speed_blend`
: Blend toward raceline speed profile when opponent odometry is stale.

`stale_timeout`
: Time after which prediction is considered stale.

`max_stale_prediction_time`
: Time after which the node stops publishing stale predictions.

`speed_profile_scale`
: Multiplier on raceline speed values.

`speed_profile_min_speed`
: Lower clamp on profile speed.

`speed_profile_max_speed`
: Upper clamp on profile speed.

`max_progress_speed`
: Maximum accepted progress-speed measurement.

`max_progress_accel`
: Maximum accepted change in measured progress speed.

`position_snap_alpha`
: How strongly fresh measurements anchor the current position estimate.

`lateral_offset_alpha`
: Filter gain for measured lateral offset.

`lateral_offset_decay_time`
: Time constant for predicted lateral offset to decay back toward the raceline.

`kf_process_var_s`
: Process noise for raceline progress.

`kf_process_var_v`
: Process noise for progress speed.

`kf_measurement_var_s`
: Measurement noise for projected raceline progress.

`kf_measurement_var_v`
: Measurement noise for progress speed.

### LiDAR Detector Parameters

`scan_topic`
: LaserScan input, usually `/scan`.

`ego_odom_topic`
: Ego pose source used to transform scan points into `map`.

`map_topic`
: Static occupancy map used to reject wall points.

`detected_odom_topic`
: Odom-like detection output consumed by the predictor.

`opponent_length`, `opponent_width`
: Approximate opponent car dimensions used to correct from visible face centroid to car center.

`max_center_correction`
: Maximum allowed centroid-to-center correction.

`min_detection_range`, `max_detection_range`
: Range gate for candidate scan points.

`wall_inflation_radius`
: Static map inflation radius for wall rejection. Increase if walls leak through; decrease if the opponent is being swallowed by nearby walls.

`cluster_tolerance`
: Euclidean point clustering radius.

`min_cluster_points`, `max_cluster_points`
: Cluster size gate.

`min_cluster_extent`, `max_cluster_extent`
: Physical cluster extent gate.

`max_raceline_projection_dist`
: Maximum distance from raceline for a valid opponent cluster. This is currently one of the most important tuning knobs.

`max_candidate_jump`
: Maximum allowed center jump when a recent detection exists.

`continuity_gate_timeout`
: Time window where jump and speed continuity checks are enforced.

`max_detection_speed`
: Maximum physically plausible opponent progress speed from detector outputs.

`detection_publish_rate_hz`
: Rate limit for detector odometry. Lower values reduce velocity spikes from tiny time deltas.

## Published Topics

`/opponent/detection_odom`
: Raw LiDAR-based opponent detection as `nav_msgs/Odometry`.

`/opponent/odom`
: Current filtered opponent estimate as `nav_msgs/Odometry`.

`/opponent/predicted_path`
: Future opponent path as `nav_msgs/Path`.

`/opponent/markers`
: Foxglove/RViz markers for current opponent estimate and future prediction.

`/opponent/detection_markers`
: Foxglove/RViz markers for dynamic scan points and selected LiDAR detection.

`/opponent/debug`
: Debug vector:

$$
\begin{bmatrix}
s_{\text{proj}} &
v_{\text{hat}} &
v_{\text{progress}} &
v_{\text{twist}} &
v_{\text{profile}} &
d_{\text{proj}} &
e_{y,\text{hat}} &
\text{stale}
\end{bmatrix}
$$

`/opponent/detection_debug`
: Detector debug vector:

$$
\begin{bmatrix}
N_{\text{dyn}} &
N_{\text{clusters}} &
J_{\text{cluster}} &
r &
d_{\text{proj}} &
\|\Delta \mathbf{c}\| &
e_t &
e_n &
v_{\text{detect}} &
\text{map\_received}
\end{bmatrix}
$$

## Algorithm Summary

1. Receive ego odometry and static map.
2. Convert LiDAR scan points into the map frame.
3. Remove points that land on static map obstacles.
4. Cluster remaining dynamic points.
5. Reject clusters with implausible size, shape, raceline distance, or continuity.
6. Correct the visible cluster centroid toward the estimated car center.
7. Publish `/opponent/detection_odom`.
8. Project the detection onto the raceline.
9. Compute progress speed from consecutive projected $s$ values.
10. Update the Kalman filter state $[s, v]^T$.
11. Snap current progress toward the fresh projected measurement for low-lag visualization.
12. Predict future progress along the raceline using filtered speed and profile-speed blending.
13. Apply lateral offset decay.
14. Publish odometry, path, markers, and debug values.
