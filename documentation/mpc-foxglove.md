# MPC visuals in Foxglove

Implemented in the repository root on `feature/mpc-foxglove`, based on `feature/mpc-autodrive`. The controller publishes standard ROS messages; no Foxglove extension or additional visualization node is needed. The layout contains a top-down map view, the latest command decision, and plots for cycle latency, published approval, and observation age.

## Run

From the repository root, build and start the existing AutoDRIVE stack as described in [mpc-autodrive.md](mpc-autodrive.md):

```bash
bash documentation/run-mpc-autodrive.sh build
bash documentation/run-mpc-autodrive.sh run
```

The visuals also work while autonomy is disabled. Starting the stack does not start the simulator/API or enable driving.

In another terminal, install the host bridge once if it is missing, then start it:

```bash
sudo apt install ros-jazzy-foxglove-bridge
bash documentation/run-mpc-autodrive.sh foxglove
```

In Foxglove, open a Foxglove WebSocket connection to `ws://localhost:8765`. Import [src/robot/mpc_controller/config/foxglove.json](../src/robot/mpc_controller/config/foxglove.json) through **Layouts → Import from file**. The file is also installed under the `mpc_controller` package's `config` directory. See the [official bridge instructions](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md) and [layout documentation](https://docs.foxglove.dev/docs/visualization/layouts).

The bridge command uses the stack's ROS domain 0 and UDP DDS profile. It binds to localhost, limits its executor to two threads, and requests subscription queue depth one. Remote connections can use an SSH tunnel, or explicitly override `address` through the command's launch arguments. Stop any bridge already using port 8765 or select another `port`. The bridge is a separate process and is not started by the controller launch.

For the matching-model fixture, build the `prototype.json` profile as described in the [package README](../src/robot/mpc_controller/README.md), then use `ros2 launch mpc_controller prototype.launch.py scene:=obstacle`. Connect a bridge in the same ROS domain. Both launches default to `visualization:=true`; use `visualization:=false` to disable the three debug topics for a baseline timing run. This does not disable existing command, status, metrics, or accepted-path outputs.

## What the views mean

| View | Topic / marker namespace | Meaning |
|---|---|---|
| Blue raceline | `/mpc/visualization/track`, `raceline` | Samples the current periodic cubic Frenet reference, including lap closure. Station remains cumulative straight segment length. This is the actual controller reference, including Porto's provisional guide. |
| White legal boundaries | `/mpc/visualization/track`, `legal_boundary_0`, `legal_boundary_1` | Closed loops used by `Reference`: explicit measured loops when supplied, otherwise the existing reconstructed width-offset loops. Loop indices do not imply left/right. |
| Raw costmap | `/costmap` | The existing input, including occupied, free, and unknown cells. In Porto this includes the static prior; it is not purely live observed free space. `/costmap/lidar` is an optional hidden layer before prior fusion. |
| Green legal search overlay | `/mpc/visualization/search_space` | Cells marked safe by the actual scene mask after track restrictions and full-cell disk inflation. Safe is 0; unavailable is 100. The layout makes unavailable/unknown cells transparent so the raw costmap remains visible. |
| Green / orange proposal | `/mpc/visualization/cycle`, `approved_proposal` / `unapproved_proposal` | Most recent finite optimizer trajectory. Green passed independent validation **and the command publication gate** for that cycle; orange was not authorized. This is a rear-axle path, not a body outline or measured execution. |
| Yellow seed route | `/mpc/visualization/cycle`, `seed_route_rear_axle` | Retained preparation route, including the bounded static-obstacle repair when needed. |
| Thin gray rollout | `/mpc/visualization/cycle`, `warm_start_rear_axle` | Per-stage seed state guess passed to optimization. |
| Cyan corridor rectangles | `/mpc/visualization/cycle`, `solver_corridors_covering_center` | Stage half-planes reconstructed in `map`, including the solver's adaptive contraction of at most 2 mm. These constrain the **covering-circle center**, not the rear axle. |
| Blue vehicle body | `/mpc/visualization/cycle`, `vehicle_footprint` | Configured oriented body at the latest fresh measured rear-axle pose, using compiled model dimensions. |
| Orange safety circle | `/mpc/visualization/cycle`, `covering_circle_plus_clearance` | Body-covering radius plus configured clearance and the existing 5 mm geometric margin, centered ahead of the rear axle. It is not calibrated uncertainty. |
| Floating decision | `/mpc/visualization/cycle`, `command_status` | Approval/rejection reason, core/solver time when available, and stale-state warning. `MPC core` excludes ROS publication and scene rebuilding; use the full-cycle plot for the controller deadline. |
| Accepted path | `/mpc/predicted_path` | Existing `nav_msgs/Path`, hidden by default to avoid duplicating the proposal. Empty on stop/rejection. |

The search mask is a conservative set of **circle-center positions**, not the full dynamically reachable set. The A* seed search adds its existing 0.20 m padding. Near a cell edge the controller can certify a safe continuous point against original cells and boundary segments even when the coarse mask does not mark that cell safe. A valid corridor can therefore extend outside the green raster. The overlay explains one cached mask; it is not an independent collision certificate or a reason to accept a command. Corridors are individual stage constraints; their visible union does not certify between-stage motion. Independent swept validation still checks original geometry.

Generated geometry uses `map`, metres, and radians. Grid axes, orientation, cell order, and observation timestamps are retained after the controller's observation-time transform. Tiny positive Z offsets separate lines; they have no physical meaning. Reference waypoint Z stores speed guidance here and is deliberately not rendered as elevation. The original costmap is frozen at its observation transform (`frameLocked: false`); standard MarkerArray, Path, and OccupancyGrid support is described in the [Foxglove 3D documentation](https://docs.foxglove.dev/docs/visualization/panels/3d).

## Freshness, timing, and resource use

Static track delivery is reliable/transient-local, depth one, on reference changes; late subscribers receive it. Cycle markers and the search overlay use best effort, depth one, at 5 Hz. Existing `/mpc/predicted_path` now also uses **best effort, depth one**, and is built only when subscribed; custom subscribers that previously requested reliable delivery must request best effort. It publishes on approved cycles and clears on stops.

Each cycle-marker sample deletes the previous sample; dynamic markers also have a 400 ms lifetime. Stale states remove the body and safety circle. Missing/stale grids replace the search overlay with a one-cell unknown grid, transparent in this layout. Candidate/route/corridor markers require fresh state and grid, matching scene/reference versions, and a recent result. Enable heartbeat and manual ownership are rechecked before displaying approval. An expired approval becomes `NoCurrentApprovedCycle`. Visuals can lag a gate change by one 200 ms visual period and are not a real-time safety indicator. Marker expiry also depends on the viewer's playback clock; interpret stopped streams and paused simulation using status/timestamps.

One existing worker still owns the solver. The visual timer has its own mutually exclusive callback group. Three executor threads allow input preparation and visual serialization while leaving a thread available for scheduling/watchdog callbacks; this does not reserve CPU time or guarantee deadlines. After command publication, the worker retains immutable paired Scene/Reference snapshots and copies candidate/route data only with a cycle-marker subscriber. The visual timer briefly snapshots data under the existing mutex, then constructs and publishes grids/markers outside it. It reuses inflation, routes, boundaries, and solver faces. Track sampling is cached between reference changes and capped at 40,000 intervals. Existing capacities bound scene size; message building still allocates and subscribers consume CPU/DDS resources.

The layout plots `/mpc/metrics.data[0]` (release through command/path/status publication calls and visual snapshot copying) and `[2]` (solver), in milliseconds, with a 20 ms reference line. The full metric excludes asynchronous visual serialization, DDS delivery, mux transit, and applied motion. Scheduling contention affects it indirectly; benchmark visuals on/off under the intended workload. Approval uses `[4]`; state/grid observation ages use `[7]`/`[8]` in seconds. Metrics are emitted for completed worker jobs; watchdog-only stops appear in `/mpc/status` and cannot be inferred solely from the approval plot. Indexed fields and numeric reference lines follow the [Plot documentation](https://docs.foxglove.dev/docs/visualization/panels/plot).

## Validation and remaining checks

On the local Ubuntu 24.04/ROS Jazzy x86 host, the ROS build passes, all 21 C++ tests pass, and all seven live ROS tests pass. Coverage includes flat/closed track rendering, rotated grid axes and exact mask contents, local-to-map proposal/corridor geometry, exact face contraction, circle placement, invalid trajectory rejection, transient-local delivery, enabled proposals, stale overlay/path/marker removal, disabled debug publishers, and existing enable/manual/emergency/deadline/clock regressions.

```bash
source /opt/ros/jazzy/setup.bash
source install-ros/setup.bash
ctest --test-dir build-ros/mpc_controller --output-on-failure
ROS_LOG_DIR="$PWD/log-ros/foxglove-tests" \
  FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/robot/mpc_controller/config/fastdds_udp.xml" \
  python3 src/robot/mpc_controller/test/test_runtime.py
```

The live suite uses an isolated ROS domain. The host bridge is not installed here. Actual Foxglove GUI rendering, layout import, slow-client behavior, and Orin shared-workload performance have not been exercised. Verify those before drawing conclusions about latency overhead. This feature changes diagnostics; it does not establish physical braking behavior or calibrate the vehicle.
