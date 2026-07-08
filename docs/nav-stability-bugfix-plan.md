# Nav Stability Bug-Fixing Plan

Status: **draft** — plan first, then implement. Branch: `plan/nav-stability-fixes`
(based on `feature/waypoint-tool`).

## Goal

Make mapping + point-to-point navigation reliable enough to build the agentic
"go to the vending machine / deliver to a specific spot" behavior on top. This
document is the fix backlog; precise docking (AprilTag/visual servo) and the
agent layer are **out of scope here** (tracked as follow-ups at the bottom).

## How to use this doc

- Each item has: symptom → root cause (file:line) → fix → how to verify → risk.
- Do them in priority order (P0 → P1 → P2). Re-test after each P0.
- Every change is a config/wiring edit; keep them as separate commits so any
  one can be reverted independently on the hardware day.

## Priority legend

- **P0** — likely causes of the current instability / safety. Do first.
- **P1** — behavioral quality; do once P0 is stable.
- **P2** — cleanup and robustness; opportunistic.

---

## P0 — correctness & safety

### P0-1. `use_sim_time: True` on planner & behavior servers (real robot has no `/clock`)
- **Symptom:** planning stalls, recoveries never fire, intermittent "transform
  timeout" / frozen goals.
- **Root cause:** `go2_robot_sdk/config/nav2_params.yaml:290` (`planner_server`)
  and `:343` (`behavior_server`) set `use_sim_time: True` while every other node
  is `False`. With no `/clock` publisher their clocks sit at 0 and TF time
  lookups against wall-time frames fail.
- **Fix:** set both to `False`. Audit the whole file so all nodes are `False` on
  hardware.
- **Verify:** `ros2 param get /planner_server use_sim_time` → `False`; send a
  `NavigateToPose` goal and confirm a path is produced and a recovery (e.g.
  spin) can trigger.
- **Risk:** none on real hardware (this is the correct value).

### P0-2. Joystick cannot override Nav2 (no manual takeover / e-stop)
- **Symptom:** can't safely grab control while autonomous; joystick "fights"
  Nav2 instead of overriding it.
- **Root cause:** in `go2_robot_sdk/launch/robot.launch.py`
  `create_teleop_nodes()` the `teleop_twist_joy` node has **no** topic remap, so
  it publishes to the default `/cmd_vel` — the same input Nav2 uses (twist_mux
  `navigation`, priority 5) — instead of the high-priority `cmd_vel_joy`
  (priority 10) defined in `config/twist_mux.yaml`.
- **Fix:** add `remappings=[('cmd_vel', 'cmd_vel_joy')]` to the `teleop_twist_joy`
  node. Confirm a joystick deadman/enable button is configured.
- **Verify:** drive Nav2 to a goal, hold the joystick — robot should immediately
  obey the stick; release and Nav2 resumes after the mux timeout.
- **Risk:** low. This is a prerequisite for safely testing everything else.

### P0-3. Control-loop rate vs. velocity ceilings mismatch
- **Symptom:** overshoot, weaving, oscillation around the path and at goals.
- **Root cause:** `nav2_params.yaml:121` `controller_frequency: 3.0` Hz combined
  with `:152-156` `max_vel_x/max_speed_xy: 3.0` m/s and `max_vel_theta: 3.0`
  rad/s. At 3 Hz the robot travels ~1 m between updates; 3 rad/s ≈ 172°/s.
- **Fix (start conservative):** `max_vel_x`/`max_speed_xy` ≈ 0.6, `max_vel_theta`
  ≈ 1.0, then raise `controller_frequency` to 10–15 Hz. Tune up from there.
- **Verify:** smooth approach, minimal overshoot at the goal; CPU headroom for
  the higher controller rate.
- **Risk:** low. Lower speed is safer for hardware testing regardless.

---

## P1 — behavioral quality

### P1-1. Planner/platform mismatch (car-like plans for a holonomic robot)
- **Symptom:** looping, curved, or reversing approach paths where the dog should
  just turn in place and walk; awkward maneuvers near goals.
- **Root cause:** `nav2_params.yaml:293` `SmacPlannerHybrid` with `:301`
  `motion_model_for_search: "REEDS_SHEPP"`, `:306` `minimum_turning_radius: 0.30`,
  `:307` `reverse_penalty: 2.1` — nonholonomic (car) planning. Meanwhile DWB has
  `:153` `max_vel_y: 0.0` (never strafes) though AMCL uses `OmniMotionModel`.
- **Fix (pick one, test):** (a) switch planner to `SmacPlanner2D` (holonomic,
  turn-in-place friendly), or (b) keep Hybrid but lower `minimum_turning_radius`
  and reduce penalties. Optionally allow small `max_vel_y` and add
  strafe-capable controller (see P1-2).
- **Verify:** planned paths are direct; no gratuitous reversing.
- **Risk:** medium — retune goal approach after switching.

### P1-2. Controller choice (DWB vs. MPPI/RPP)
- **Symptom:** jerky following, poor recovery from small deviations.
- **Root cause:** DWB tuned for diff-drive defaults; `max_vel_y: 0.0`.
- **Fix (optional, after P1-1):** evaluate `RegulatedPurePursuit` (simple,
  robust for waypoint following) or `MPPI` (smooth, holonomic-aware). Keep DWB
  as fallback.
- **Verify:** smoother tracking on the vending-machine route; fewer oscillations.
- **Risk:** medium — new controller needs its own tuning pass.

### P1-3. Costmap inflation mismatch (global vs. local)
- **Symptom:** global path hugs walls but local controller wants more clearance
  → oscillation / wall scraping in corridors and doorways.
- **Root cause:** local `nav2_params.yaml:226-227` `inflation_radius: 0.55`,
  `cost_scaling_factor: 3.0` vs. global `:269-270` `inflation_radius: 0.25`,
  `cost_scaling_factor: 1.0`.
- **Fix:** unify inflation (start ~0.45 both) and align `cost_scaling_factor`
  (~2.5). Keep footprint (`0.36`/`-0.45` × `±0.20`) consistent.
- **Verify:** global path keeps a sensible standoff from walls that the local
  controller can actually follow.
- **Risk:** low/medium — too-large inflation can block tight legitimate gaps
  (relevant to docking; see follow-ups).

### P1-4. Oversized global costmap (500 m × 500 m @ 5 cm = 100M cells)
- **Symptom:** high RAM/CPU, laggy costmap updates that starve the control loop.
- **Root cause:** `nav2_params.yaml:240-243` `width/height: 500`, `origin: -250`.
- **Fix:** size to the actual operating area (e.g. 40 × 40 m) or set
  `downsample_costmap`/coarser resolution for the global layer.
- **Verify:** lower `nav2` process memory; costmap update rate holds under load.
- **Risk:** low — ensure the map fits the chosen bounds.

### P1-5. Odometry is single-source, covariance-free, unfused
- **Symptom:** pose "jumps" propagate into TF; AMCL/SLAM get no uncertainty
  signal so they can't weight scans well.
- **Root cause:** `odom`/TF come solely from the robot's `rt/utlidar/robot_pose`
  (`domain/constants/webrtc_topics.py:21`, published in
  `infrastructure/ros2/ros2_publisher.py`), with zero covariance and a baked-in
  `+0.07` z offset; IMU (`imu`) is published but not fused.
- **Fix:** add a `robot_localization` EKF fusing `odom` + `imu`; publish
  `odom→base_link` from the EKF; set realistic covariances on the source
  Odometry. (Bigger change — schedule deliberately.)
- **Verify:** smoother `odom→base_link`; AMCL converges faster and holds.
- **Risk:** medium/high — new node + TF ownership change; test in isolation.

---

## P2 — cleanup & robustness

### P2-1. Separate SLAM vs. navigation modes
- **Symptom:** waypoints captured in `map` frame drift because live SLAM keeps
  correcting `map→odom` during navigation.
- **Root cause:** `robot.launch.py` launches SLAM **and** Nav2 together (`:84-85`
  both default `true`).
- **Fix:** map with SLAM (mapping.launch.py), save it, then **navigate** with
  AMCL + static map (navigation.launch.py). Don't run live SLAM under waypoint
  execution. Also set `set_initial_pose`/seed AMCL (`nav2_params.yaml:41`).
- **Verify:** captured waypoints stay put across a run.
- **Risk:** low — this is the intended two-mode split.

### P2-2. Two divergent LiDAR pipelines + topic-remap mismatch
- **Symptom:** Python aggregator receives nothing under `robot.launch.py`.
- **Root cause:** `lidar_processor` (Python) subscribes `/robot0/point_cloud2`,
  but single-robot driver publishes `point_cloud2` and `robot.launch.py` adds no
  remap; the C++ `lidar_processor_cpp` (used by mapping/navigation launches)
  *does* remap. Two implementations to maintain.
- **Fix:** pick one implementation; fix the subscription/remap so it matches the
  driver's `point_cloud2`.
- **Verify:** `/pointcloud/aggregated` publishes under the chosen launch.
- **Risk:** low.

### P2-3. Goal-checker tolerance inconsistency
- **Symptom:** goal "reached" behavior differs from expectation.
- **Root cause:** `general_goal_checker` `xy_goal_tolerance: 0.25`
  (`nav2_params.yaml:144`) vs. DWB `xy_goal_tolerance: 0.3` (`:173`).
- **Fix:** make them consistent. Note: ~0.25–0.3 m is fine for transit but too
  loose for docking — the last meter is handled by the vision follow-up.
- **Risk:** low.

### P2-4. Dead code in `handle_cmd_vel`
- **Symptom:** none (confusing only).
- **Root cause:** `application/services/robot_control_service.py` builds
  `gen_mov_command(...)` into `_` then re-generates it in the adapter.
- **Fix:** remove the dead call.
- **Risk:** none.

### P2-5. Confirm firmware obstacle-avoidance mode is intentional
- **Symptom:** robot's onboard avoidance (sport api `1003`) can fight Nav2's
  `cmd_vel` in tight spots.
- **Root cause:** driver switches to `OBSTACLE_AVOIDANCE_TOPIC` when the
  `obstacle_avoidance` param is on (`application/utils/command_generator.py:110`).
- **Fix:** decide policy — for Nav2-driven transit keep it **off**; reserve
  firmware avoidance for a "dumb-but-safe" fallback mode.
- **Risk:** low.

---

## Suggested execution order (hardware day)

1. P0-1, P0-2, P0-3 (config + launch remap). Rebuild, re-test localization and a
   couple of short goals before proceeding.
2. P2-1 (map once, navigate on AMCL). Capture vending/delivery waypoints with
   `waypoint_tool`.
3. P1-3, P1-4 (inflation + costmap size). Re-test corridor/doorway.
4. P1-1 (planner) → optionally P1-2 (controller). Re-tune goal approach.
5. P1-5 (EKF) and P2-* cleanups as time allows.

## Test protocol

- **T1 Localization:** on saved map, seed AMCL pose, drive manually, confirm
  `map→base_link` is stable (no jumps) in RViz/Foxglove.
- **T2 Short goal:** `NavigateToPose` 2–3 m ahead; expect direct path, smooth
  stop within tolerance.
- **T3 Route:** `waypoint_tool run --only vending` then a delivery point; expect
  no oscillation in the doorway/near furniture.
- **T4 Safety:** joystick override interrupts Nav2 mid-run (P0-2).

## Out of scope here (follow-ups — separate branches)

- **Precise docking (the "last meter"):** AprilTag on vending machine + delivery
  spot, or monocular visual servoing, since Nav2 goal tolerance (~0.25 m) is too
  loose to reliably deliver to an exact spot. High value.
- **Agentic orchestration:** LLM/VLM agent exposing `go_to_waypoint`, `dock`,
  `nudge(cmd_vel)`, `say` (existing `/tts`), and sport commands via `webrtc_req`;
  VLM-in-the-loop for recovery. Nav2 stays the metric-nav backbone.
- **Camera-augmented obstacles:** flag low/glass obstacles the flattened `/scan`
  misses.
