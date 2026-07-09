#!/usr/bin/env bash
#
# Save the live SLAM map (2D occupancy grid) to ./maps/<name>.pgm + .yaml.
# Run this WHILE run_mapping.sh (or run_mapping_nav.sh) is running, once you've
# driven around enough that the map looks complete.
#
# This is the map Nav2 AND the sim consume - NOT the 3d_map.ply (that's the raw
# point cloud). Replays 1:1 in the sim afterwards.
#
# Usage:
#     scripts/save_map.sh my_map
#
set -euo pipefail
cd "$(dirname "$0")/.."

NAME="${1:?Usage: scripts/save_map.sh <name>}"
if ! docker ps --format '{{.Names}}' | grep -qx go2_robot; then
  echo "No running mapping session (container 'go2_robot')." >&2
  echo "Start one first:  ROBOT_IP=... scripts/run_mapping.sh" >&2
  exit 1
fi

# Run map_saver in the SAME container as slam_toolbox, so it's guaranteed on the
# same ROS graph and can see /map. Output lands in the mounted ./maps dir.
docker exec go2_robot bash -c "
  source /opt/ros/\$ROS_DISTRO/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 run nav2_map_server map_saver_cli -f /ros2_ws/maps/$NAME --ros-args -p save_map_timeout:=10.0
"

echo ""
echo "Saved maps/$NAME.yaml + maps/$NAME.pgm"
echo "Replay this exact map in the sim:"
echo "  SIM_MAP=/ros2_ws/src/maps/$NAME.yaml SIM_START_X=0 SIM_START_Y=0 \\"
echo "    docker compose -f docker/docker-compose.yml up sim"
