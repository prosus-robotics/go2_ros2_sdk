#!/bin/bash
# Build + launch the Tier-0 Nav2 test harness (go2_nav_sim) inside the container.
# Used by the `sim` service in docker-compose.yml.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Build into a SEPARATE base (sim_build/sim_install) so we never collide with the
# image's prebuilt /ros2_ws/build, which was baked WITHOUT --symlink-install
# (a plain colcon build there leaves real directories, and --symlink-install then
# fails trying to replace them with symlinks). A fresh base sidesteps that and
# lets host edits to go2_nav_sim / go2_robot_sdk (params!) be picked up on restart.
cd /ros2_ws
colcon build --symlink-install \
  --packages-select go2_interfaces go2_robot_sdk go2_nav_sim \
  --build-base /ros2_ws/sim_build \
  --install-base /ros2_ws/sim_install

source /ros2_ws/sim_install/setup.bash

# (Re)generate the built-in example world into the installed share dir.
python3 src/go2_nav_sim/tools/make_example_world.py \
    /ros2_ws/sim_install/go2_nav_sim/share/go2_nav_sim/worlds

# Map + start pose are overridable from the host via env vars (declared on the
# `sim` service in docker-compose.yml). Default = the generated example world.
# The repo root is mounted at /ros2_ws/src, so a map you drop in the repo is at
# /ros2_ws/src/<name>.yaml inside the container.
#   SIM_MAP=/ros2_ws/src/my_map.yaml SIM_START_X=0 SIM_START_Y=0 \
#     docker compose -f docker/docker-compose.yml up sim
SIM_MAP="${SIM_MAP:-/ros2_ws/sim_install/go2_nav_sim/share/go2_nav_sim/worlds/example.yaml}"
SIM_START_X="${SIM_START_X:-2.0}"
SIM_START_Y="${SIM_START_Y:-2.0}"
SIM_START_YAW="${SIM_START_YAW:-0.0}"

echo "[sim_entrypoint] map=${SIM_MAP} start=(${SIM_START_X}, ${SIM_START_Y}, ${SIM_START_YAW})"
exec ros2 launch go2_nav_sim sim_bringup.launch.py \
    rviz:=false foxglove:=true \
    map:="${SIM_MAP}" \
    start_x:="${SIM_START_X}" start_y:="${SIM_START_Y}" start_yaw:="${SIM_START_YAW}"
