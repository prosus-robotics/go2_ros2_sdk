# go2_nav_sim — Tier-0 kinematic Nav2 test harness

Test the Go2 **Nav2 stack** (planner / controller / costmaps / behaviors +
`twist_mux` priority/deadman) **without Gazebo, without a GPU**, on any machine
with ROS 2 Humble.

The real driver abstracts the dog as `cmd_vel_out → motion`, returning `odom` +
`/scan`. This package replaces the robot + LiDAR pipeline with a tiny simulator:

- **`kinematic_sim`** — integrates `cmd_vel_out` into `odom` + the
  `odom→base_link` TF, and raycasts the `/map` occupancy grid into a
  `sensor_msgs/LaserScan` that matches the real scan geometry. It seeds AMCL
  once from the known start pose, and can inject odom drift to stress
  localization / the EKF (FIX-7).
- **`scenario_runner`** — drives a set of navigation goals via
  `nav2_simple_commander` and prints a **stability scorecard**: success,
  recoveries, min obstacle clearance, command oscillation, and final pose error.

It reuses the **actual** `go2_robot_sdk/config/nav2_params.yaml` and
`twist_mux.yaml`, so what you validate here is exactly what runs on the robot.

> Runs in wall-clock time, so `use_sim_time: False` (FIX-2) is used unchanged —
> no `/clock`. This is the one big difference from a Gazebo/Isaac sim, where you
> *must* flip `use_sim_time:=true`.

## Run on macOS (recommended) — via Docker

You do **not** need a Linux machine. Nav2 is ROS 2 (Linux-only), but everything
runs inside the one Linux container defined in `docker/docker-compose.yml`,
which works on Docker Desktop for Mac (native on Apple Silicon). It's headless —
you view it in **Foxglove Studio** in your browser.

```bash
# from the repo root. First run compiles the workspace (a few minutes), then caches.
docker compose -f docker/docker-compose.yml up --build sim
```

Open **Foxglove Studio** → "Open connection" → `ws://localhost:8765`. In a **3D**
panel, add these topics:

| topic | shows |
|---|---|
| `/map` | the occupancy grid |
| `/scan` | the synthetic LiDAR |
| `/nav_sim/robot_model` | **the Go2** (dark body + yellow heading arrow) |
| `/plan`, `/local_plan` | the global / local path |
| `/nav_sim/waypoint_markers` | the route you draw (numbered) |

The robot model is a marker locked to `base_link`, so it rides the TF and always
shows where the robot actually is and which way it faces.

### Set goals & waypoints from Foxglove (the click-drag arrow)

Foxglove can't send Nav2 *actions* directly (it only publishes topics), so the
bringup includes a tiny `goal_relay` node that turns published poses into Nav2
goals. In the 3D panel, click the **"Publish"** tool (the pose arrow), then set
its topic in the panel settings:

- **Publish `geometry_msgs/PoseStamped` → `/goal_pose`** → the robot drives there
  now (this is the exact equivalent of RViz's "Nav2 Goal"). Click-drag to also
  set the final heading.
- To build a **multi-point route**: point the Publish tool at `/waypoint_add`
  instead and drop several poses (each appears as a numbered marker). Then add a
  **Publish** *button* panel that sends `std_msgs/Empty` to `/waypoints_run` to
  execute the route, and one to `/waypoints_clear` to reset.

Or run the automated **scorecard** in a second terminal:

```bash
docker compose -f docker/docker-compose.yml exec sim \
  bash -lc "source /ros2_ws/sim_install/setup.bash && ros2 run go2_nav_sim scenario_runner"
```

### Waypoints via the CLI tool

The full Nav2 `FollowWaypoints` action runs in the sim, so the existing
`waypoint_tool` works exactly as on the robot. Drive the robot to a spot (send a
Nav2 goal from Foxglove, or `ros2 topic pub --once /goal_pose ...`), then:

```bash
docker compose -f docker/docker-compose.yml exec sim bash -lc \
  "source /ros2_ws/sim_install/setup.bash && ros2 run go2_robot_sdk waypoint_tool capture vending"
# ... capture more, then replay the route:
docker compose -f docker/docker-compose.yml exec sim bash -lc \
  "source /ros2_ws/sim_install/setup.bash && ros2 run go2_robot_sdk waypoint_tool run"
```

Your source is bind-mounted, so editing Python on the Mac is live
(`--symlink-install`); just restart the `sim` service to reload. To use your own
map instead of the generated one, drop it in the repo and pass `map:=...` by
editing the `command:` in the compose service (or run the launch by hand inside
the container).

> Why not run MuJoCo/Gazebo natively on the Mac and bridge to Nav2? Docker
> Desktop on macOS is a VM, so `network_mode: host` / cross-boundary DDS
> discovery is unreliable. Keeping *everything in one container* avoids that —
> which is exactly what this headless harness is built for.

## Native (Linux) build

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-twist-mux ros-humble-foxglove-bridge
pip3 install numpy   # or: sudo apt install python3-numpy

cd ~/ros2_ws            # your workspace containing this repo
colcon build --packages-select go2_nav_sim
source install/setup.bash
```

## 1. Generate the built-in test world (once)

A 20×20 m room with a 1.2 m doorway and a box obstacle, matching the scenarios:

```bash
python3 src/go2_ros2_sdk/go2_nav_sim/tools/make_example_world.py \
        install/go2_nav_sim/share/go2_nav_sim/worlds
```

(Adjust the path to wherever `go2_nav_sim` is installed. You can also point the
generator anywhere and pass `map:=` explicitly.)

## 2. Interactive run (RViz, native Linux)

```bash
ros2 launch go2_nav_sim sim_bringup.launch.py
```

Then send goals from RViz ("Nav2 Goal") and watch the path, costmaps, and the
robot track it. Use your own map instead:

```bash
ros2 launch go2_nav_sim sim_bringup.launch.py \
    map:=/abs/path/office_map_v1.yaml start_x:=1.0 start_y:=0.5 start_yaw:=0.0
```

## 3. Automated scorecard

In a second terminal (with the bringup running):

```bash
ros2 run go2_nav_sim scenario_runner
# subset only:
ros2 run go2_nav_sim scenario_runner --ros-args -p scenarios:="doorway,around_box"
```

Example output:

```
scenario     result     t(s) recov clear(m)  osc  xy_err yaw_err  verdict
------------------------------------------------------------------------
straight     OK          9.4     0     0.71    2    0.08    0.05   PASS
doorway      OK         21.7     1     0.28    6    0.14    0.09   PASS
around_box   OK         12.1     0     0.33    4    0.11    0.07   PASS
rotate       OK          6.8     0     2.10    3    0.05    0.04   PASS
------------------------------------------------------------------------
TOTAL: 4/4 scenarios passed
```

Metrics map to the bugs we fixed:

| metric | what it catches |
|---|---|
| `clear(m)` | wall-scraping (min LiDAR range during the run) |
| `osc` | overshoot/weave (cmd_vel angular sign flips) |
| `recov` | stuck / thrashing |
| `xy_err`, `yaw_err` | goal accuracy |

## 4. Stress localization / EKF (FIX-7)

Inject odom drift so AMCL/EKF must correct it, and A/B the result:

```bash
ros2 launch go2_nav_sim sim_bringup.launch.py odom_yaw_noise_std:=0.05 odom_xy_noise_std:=0.02
```

## Limitations (by design)

- First-order velocity tracking with **acceleration limits** (`accel_x/y/theta`,
  matched to `nav2_params` `acc_lim_*`) but **no legged dynamics** — no gait,
  slip, or body sway (that's Tier 1: Gazebo/CHAMP). The accel model matters:
  without it a purely instantaneous kinematic base makes DWB (which predicts
  trajectories using acceleration limits) overshoot and oscillate badly.
- 2D raycast scan (no z, no clutter noise) unless you inject it.
- Localization drift only appears if you enable the noise params.

For legged realism build a Tier-1 (Gazebo + CHAMP) or Tier-2 (Isaac) sim; this
harness is the fast inner loop for tuning and regression-guarding nav config.
