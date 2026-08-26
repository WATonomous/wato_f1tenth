## MPPI notes for race
- Raceline generation format, mppi has a specified format and things it needs, specifically needs each waypoint's distance to left and right wall

## Problems (Already Solved)

### 1. When to fire control step callback in mppi node
- Problem with running control step on pf pose callback natively: If pf stalls, control step stalls with it
- Problem with running control step on pure timer callback: For single threaded executor, ROS2 designed such that it fires timer callbacks first. If a timer callback (control step) executed longer than the time gap between timer callback rate, the single thread will continously only execute timer callback, without running any other callbacks. This results in pf pose callback never updating the latest pose in mppi, thereby control step always using the old, not updated pose
- Solution proposed and used: Back to firing control step during pf pose callback, with a rate limiter of minimum time gap (control loop hz), plus a watchdog that fires a control step if pf pose stalled for some time (control_watchdog_max_silence_sec ), the watchdog checks if pf stalled at some rate (control_watchdog_hz)
- Another solution would be using multithreaded executor with pure timer callback for control step, such that control step and pf pose callback and run in parallel, but then we would need to handle race conditions of using the same variable (latest_pose), one is reading, one is writing

### 2. Small problem with mutiple subscribers to pf/pose/odom
- When opponent predictor subscribes to pf/pose/odom, along side mppi node, it may decrease the rate of the particle filter updating
- Solution proposed but NOT used: a seperate publisher by mppi node that publishes to opponent predictor, so pf/pose/odom only has one subscriber (mppi node)

### 3. Reliable QoS on debug topics, subscribed over network (Not really a problem with Wato stack)
- Reliable means publisher ensures subscriber receives the message, if network bad and subscriber did not acknowledge, it tries to resend messages. During this process it keeps unacknowledged messages in a history buffer, when this buffer is full, it BLOCKS entire callback until subscriber acknowledges. 
- Debug topic publishers are part of control loop so if QoS is reliable, it slows down the actual control topic also. 
- Solutoin: Make debug topics Best Effort, keep control publisher Reliable

### 4. Important for MPPI: When to clear warm start
- Problem: When control step took longer to execute, when do we clear the warm start of mppi.
- Solution part 1: clear the warm start when time passed between control steps bigger than time horizon, then the warm start is useless because we are passed that horizon
- Additional guard: actions are in [-1, 1], two per step (steer-rate, accel). If the first 3 steps of the plan (6 values) have ≥5 values with |a| ≥ 0.98 (mppi_guard_aopt_threshold), the plan is saturated — e.g. recovering from a slip or facing sideways to the raceline. If this happens 4 control ticks in a row (mppi_guard_saturation_callbacks), wipe the warm start so sampling restarts from 0 (with warm start at +1, samples can only reach +0.2 … +1, never negative)