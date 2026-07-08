# Nav Stability Fix Plan (careful, sequenced)

Status: **IMPLEMENTED on `plan/nav-stability-fixes`** — FIX-1…7 landed as seven
separate commits (one per fix, in order) on top of this doc. Baseline
`feature/waypoint-tool` remains pushed to `origin` for rollback; revert any
single `FIX-N:` commit to back that one fix out.

This is the careful execution plan for the seven stability issues. It is
deliberately ordered so the **safety** and **pure-correctness** fixes land
first, cheap config fixes next, and the one risky code change (EKF) last.

**Before testing:** rebuild + re-source on the robot:
`colcon build --packages-select go2_robot_sdk lidar_processor && source install/setup.bash`.
FIX-7 additionally needs `robot_localization` installed
(`sudo apt install ros-$ROS_DISTRO-robot-localization`); it only runs when you
pass `ekf:=true`, so it does not affect the default bringup.

---

## Ground rules (read before touching hardware)

1. **Baseline is safe.** `feature/waypoint-tool` is pushed to `origin`. At any
   point you can return to known-good with:
   `git checkout feature/waypoint-tool` (or `git checkout feature/waypoint-tool -- <file>`).
2. **Implemented on this branch** (`plan/nav-stability-fixes`) per request — the
   fixes sit as separate commits on top of this doc, not a separate `fix/...`
   branch.
3. **One fix = one commit.** Never stack two untested changes on the robot.
4. **Test gate after every fix** (see the per-fix "Verify" + the protocol at the
   bottom). If a fix regresses, `git revert` that single commit and move on.
5. **Record a baseline bag first:** `ros2 bag record /tf /tf_static /odom /scan
   /cmd_vel /cmd_vel_out /plan /local_plan` for ~1 min of manual driving, so
   every later change can be compared against it.
6. On real hardware **every** node must have `use_sim_time: False`.

Recommended order: **FIX-1 → FIX-2 → FIX-3 → FIX-4 → FIX-5 → FIX-6 → FIX-7.**

---

## FIX-1 (P0, safety) — Make the joystick a real override / e-stop

**Why:** `teleop_twist_joy` has no topic remap, so it publishes to `/cmd_vel`
(same as Nav2, mux priority 5) instead of `cmd_vel_joy` (priority 10). You have
no clean manual takeover. Also `require_enable_button: false` means the teleop
node streams Twist continuously — once remapped to the priority-10 input that
would *constantly* win the mux and **block Nav2 with zeros**. So the remap and a
deadman button must be done together.

**Change A — remap** in `go2_robot_sdk/launch/robot.launch.py`,
`create_teleop_nodes()`, the `teleop_twist_joy` node:

```python
Node(
    package='teleop_twist_joy',
    executable='teleop_node',
    name='go2_teleop_node',
    condition=IfCondition(with_joystick),
    parameters=[self.config.config_paths['twist_mux']],
    remappings=[('cmd_vel', 'cmd_vel_joy')],   # ADD THIS
),
```

Apply the **same remap** in `launch/navigation.launch.py` and
`launch/mapping.launch.py` (they have the identical bug).

**Change B — deadman** in `go2_robot_sdk/config/twist_mux.yaml` under
`/go2_teleop_node`:

```yaml
    require_enable_button: true
    enable_button: 4          # pick a real button index on your pad (e.g. LB)
    enable_turbo_button: -1
```

Now: hold the button (sticks centered) = commanded stop that overrides Nav2 =
**e-stop**; hold + move sticks = manual drive; release = Nav2 resumes after the
0.5 s mux timeout.

**Depends on:** nothing. Do this first.
**Verify:** `ros2 topic info /cmd_vel_joy` shows the teleop publisher; start a
Nav2 goal, press+hold the deadman → robot stops/obeys stick; release → Nav2
resumes. Confirm with `ros2 topic echo /cmd_vel_out` which source wins.
**Rollback:** remove the remap / revert twist_mux.yaml.
**Risk:** low. (This is the safety net for testing FIX-2…7.)

---

## FIX-2 (P0, correctness) — `use_sim_time: False` on planner & behavior servers

**Why:** on a real robot there is no `/clock`; those two nodes' clocks sit at 0
while everything else uses wall time → TF time lookups fail → planning failures,
frozen recoveries, timeouts.

**Change** in `go2_robot_sdk/config/nav2_params.yaml`:

- Line 290 `planner_server: use_sim_time: True` → `False`
- Line 343 `behavior_server: use_sim_time: True` → `False`

Audit the rest of the file to confirm all remaining nodes are already `False`
(amcl, controller_server, both costmaps, bt_navigator, waypoint_follower,
map_server, robot_state_publisher — they are).

**Depends on:** nothing.
**Verify:** `ros2 param get /planner_server use_sim_time` → `False`; a
`NavigateToPose` goal now produces a path and a recovery (spin/backup) can fire;
the "message filter / transform timeout" log spam stops.
**Rollback:** revert the two lines.
**Risk:** none — this is simply the correct value on hardware.

---

## FIX-3 (P0→P1) — Shrink the global costmap (do before raising control rate)

**Why:** 500 m × 500 m @ 5 cm = **100M cells**. Heavy RAM/CPU that starves the
control loop and makes every other timing problem worse. Fix this *before*
FIX-5 raises the controller frequency, so there's CPU headroom.

**Change** in `nav2_params.yaml` `global_costmap` (lines 240-243):

```yaml
      width: 40            # was 500  — size to your actual operating area
      height: 40           # was 500
      origin_x: -20.0      # was -250.0  (centered on the map)
      origin_y: -20.0      # was -250.0
```

Set width/height/origin to cover your real space with margin (measure the saved
map). Note: in AMCL + static-map mode the static layer is bounded by the map
anyway; these bounds mainly cap preallocation.

**Depends on:** nothing (but sequence it before FIX-5).
**Verify:** `nav2`/`global_costmap` process RAM drops sharply; `ros2 topic hz
/global_costmap/costmap` holds; a goal across the whole area still plans.
**Rollback:** revert the four lines.
**Risk:** low/medium — make sure the bounds actually contain your map + goals.

---

## FIX-4 (P1) — Unify costmap inflation (stop the "scrapes walls / oscillates")

**Why:** local `inflation_radius: 0.55` vs global `0.25`. The global planner
threads paths 0.25 m from walls, but the local controller wants 0.55 m clearance
→ it fights its own plan in doorways.

**Change** in `nav2_params.yaml` — make both layers match:

- Local `inflation_layer` (lines 226-227): `cost_scaling_factor: 3.0` →
  `2.5`, `inflation_radius: 0.55` → `0.45`
- Global `inflation_layer` (lines 269-270): `cost_scaling_factor: 1.0` →
  `2.5`, `inflation_radius: 0.25` → `0.45`

Sanity check against your tightest passage: `2 × inflation_radius +
robot_width (0.40) ` must be **less** than the doorway width, or the planner
will refuse it. If your doorways are tight, drop to ~0.35 and re-check.

**Depends on:** nothing.
**Verify:** global path keeps a consistent standoff the local controller can
follow; corridor/doorway oscillation reduced.
**Rollback:** revert the four values.
**Risk:** low/medium — too much inflation blocks narrow gaps (relevant later for
docking, which will need reduced inflation near the goal).

---

## FIX-5 (P0) — Sane velocities + a real control rate

**Why:** `controller_frequency: 3.0` Hz with `max_vel_x/max_speed_xy: 3.0` m/s
and `max_vel_theta: 3.0` rad/s (~172°/s). At 3 Hz the robot moves ~1 m between
updates → guaranteed overshoot/oscillation.

**Change** in `nav2_params.yaml` `controller_server` / DWB:

```yaml
    controller_frequency: 10.0      # was 3.0  (target 10–15)
...
      max_vel_x: 0.6                 # was 3.0
      max_vel_y: 0.0                 # keep 0.0 for now (revisit in FIX-6)
      max_vel_theta: 1.0             # was 3.0
      max_speed_xy: 0.6              # was 3.0
      # optional smoothing: soften accels
      acc_lim_x: 1.5                 # was 2.5
      acc_lim_theta: 2.0             # was 3.2
```

**Depends on:** FIX-3 (CPU headroom for the higher rate).
**Verify:** `ros2 topic hz /cmd_vel` ≈ `controller_frequency`; smooth approach,
minimal overshoot at the goal; CPU still has headroom.
**Rollback:** revert the block.
**Risk:** low — slower is safer for hardware testing.

---

## FIX-6 (P1) — Fix the planner/platform mismatch

**Why:** `SmacPlannerHybrid` + `REEDS_SHEPP` + `minimum_turning_radius: 0.30` +
`reverse_penalty` plans car-like/reversing paths, while DWB has `max_vel_y: 0.0`
(never strafes). The Go2 can turn in place — it should just turn and walk.

**Change (recommended first variant)** — switch planner to holonomic 2D in
`nav2_params.yaml` `planner_server` (replace the `GridBased` Hybrid block,
lines 292-322):

```yaml
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.25
      allow_unknown: false
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

`SmacPlanner2D` ignores heading in search (holonomic); DWB's `RotateToGoal`
critic handles final yaw. Keep the old Hybrid block commented out so you can
flip back in seconds.

**Strafing decision (test both):** leave `max_vel_y: 0.0` (turn-in-place, most
stable) first. Only if you want lateral micro-adjustments, try `max_vel_y: 0.3`
with `vy_samples: 10` — but quadruped strafing is less stable, so treat as
optional.

**Depends on:** FIX-5 (test at sane speeds).
**Verify:** planned paths are direct, no reversing arcs; approaches turn-then-go.
**Rollback:** restore the Hybrid block (kept commented).
**Risk:** medium — expect a short retune of goal approach.

---

## FIX-7 (P1, highest risk — consider deferring until FIX-1…6 are validated) — Fused, covariance-bearing odometry

**Why:** `odom` + the `odom→base_link` TF come solely from the robot's onboard
`rt/utlidar/robot_pose`, with **zero covariance**, and the IMU is not fused.
When that estimate hiccups, the whole TF tree jumps.

**Important constraints discovered in code (make this careful):**
- The driver publishes **both** the TF and the `Odometry` topic
  (`infrastructure/ros2/ros2_publisher.py`, `_publish_transform` lines 54-77 and
  `_publish_odometry_topic` lines 79-102), stamped with `now()` (not the sensor
  stamp) and with a baked-in `+0.07` z.
- The IMU is a **custom `go2_interfaces/IMU`** (`ros2_publisher.py:10,169-177`),
  **not** `sensor_msgs/Imu`. `robot_localization` requires `sensor_msgs/Imu`.
- Because `robot_pose` is already an onboard *fused* estimate, the EKF's value is
  mainly (a) proper covariances, (b) smoothing/outlier rejection, (c) continuity
  from IMU between pose updates — not brand-new information. **If FIX-1…6 make it
  stable, you may not need this.**

**Change (multi-part — do on its own branch, isolated):**
1. Publish a standard `sensor_msgs/Imu` (convert quaternion + gyro + accel from
   `go2_interfaces/IMU`, add covariances) — either in the driver or a tiny
   converter node.
2. Populate covariance on the `Odometry` topic (realistic pose/twist diagonals).
3. Make the driver's `odom→base_link` TF broadcast **optional** via a param
   (e.g. `publish_odom_tf`, default `True`) and turn it **off** when the EKF runs
   (only one node may own that TF).
4. Add a `robot_localization` `ekf_node` + `config/ekf.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: odom
    odom0_config: [true, true, false,  false, false, true,   false,false,false, false,false,false, false,false,false]
    imu0: imu/data                # the new sensor_msgs/Imu
    imu0_config: [false,false,false, false,false,true,  false,false,false, false,false,true,  false,false,false]
```

5. Add `robot_localization` to package deps.

**Implemented as `config/ekf.yaml` + an `ekf:=true` launch arg (default off).**
One flag does everything: `ros2 launch go2_robot_sdk robot.launch.py ekf:=true`
starts `ekf_node` **and** flips the driver's `publish_odom_tf` to `false`, so
`odom→base_link` ownership transfers cleanly (no dual broadcaster). The filter
fuses odom `x, y, yaw` + **IMU yaw-rate only** — the Go2 IMU's absolute yaw has
an arbitrary zero that would fight the odom yaw, so it is intentionally not
fused (this differs from the sketch above). Independent of the flag, the driver
now always publishes `sensor_msgs/Imu` on `imu/data` and real pose covariance
on `odom`.

**Depends on:** FIX-1…6 validated (so you isolate EKF effects).
**Verify:** exactly **one** publisher of `odom→base_link` (`ros2 run tf2_tools
view_frames`; no `TF_REPEATED_DATA`/multiple-authority warnings); smoother pose;
AMCL converges faster.
**Rollback:** set `publish_odom_tf: True` again + stop the EKF; revert. This is
why it's last.
**Risk:** high — TF ownership change, message-type conversion, timing.

---

## Test protocol (run after each fix)

- **T1 Localize:** on the saved map, seed AMCL pose, drive manually; `map→
  base_link` stable in RViz/Foxglove (no jumps).
- **T2 Short goal:** `NavigateToPose` 2–3 m ahead → direct path, smooth stop in
  tolerance.
- **T3 Route:** `waypoint_tool run --only vending` then a delivery point → no
  doorway oscillation.
- **T4 Safety:** deadman override interrupts Nav2 mid-run and resumes on release.

## Branch & commit strategy

```
feature/waypoint-tool  (pushed, baseline)
  └─ fix/nav-stability   (new; one commit per FIX-n)
       FIX-1 …            → test → FIX-2 → test → …
```
After hardware validation, open a PR from `fix/nav-stability`.

## Strongly recommended companion (not in the 7, but interacts)

Run **mapping and navigation as separate modes**: build+save the map with SLAM,
then navigate with **AMCL + static map** (`navigation.launch.py`) rather than
live SLAM under Nav2 (`robot.launch.py` runs both). Live SLAM keeps correcting
`map→odom`, so waypoints captured in `map` drift mid-run. This makes FIX-1…7
much easier to evaluate. (Tracked with the other cleanup items below.)

## Deferred to a later pass (the remaining backlog)

- Consolidate the two LiDAR pipelines (Python `lidar_processor` vs C++
  `lidar_processor_cpp`) + fix the `/robot0/point_cloud2` vs `point_cloud2`
  remap mismatch in `robot.launch.py`.
- Unify goal tolerances (`general_goal_checker` 0.25 vs DWB 0.30).
- Remove dead code in `handle_cmd_vel`.
- Define firmware obstacle-avoidance policy (keep sport api `1003` off for Nav2
  transit; reserve as a fallback).
- Follow-ups for the end goal: AprilTag/visual-servo docking for the precise
  "last meter," and the agentic orchestration layer.
