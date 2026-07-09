#!/usr/bin/env bash
#
# Capture / list / run NAMED Nav2 waypoints while a robot session is live
# (run_nav.sh or run_mapping_nav.sh -> container 'go2_robot'). Runs the
# waypoint_tool INSIDE that container so it shares the ROS graph and TF.
#
# How waypoints are defined: "capture" saves the robot's CURRENT pose (position
# AND heading) under a name. So you drive the dog to the spot, face it the way
# you want it to arrive, and capture. That's your "table", "vending", etc.
#
# Usage (run these in a 2nd terminal while the robot session runs):
#     scripts/waypoint.sh capture table        # save current spot as "table"
#     scripts/waypoint.sh capture vending
#     scripts/waypoint.sh list                  # show what's saved
#     scripts/waypoint.sh show                  # Foxglove markers -> /waypoint_markers
#     scripts/waypoint.sh run --only table      # drive to just "table"
#     scripts/waypoint.sh run                   # drive ALL of them, in order
#     scripts/waypoint.sh remove table
#
# Waypoints persist to ./maps/<WAYPOINTS>.yaml on your Mac (default: waypoints).
# Keep a set per map, e.g.:  WAYPOINTS=test_map_1_wp scripts/waypoint.sh list
#
set -euo pipefail
cd "$(dirname "$0")/.."

WAYPOINTS="${WAYPOINTS:-waypoints}"

if ! docker ps --format '{{.Names}}' | grep -qx go2_robot; then
  echo "No running robot session (container 'go2_robot')." >&2
  echo "Start one first, e.g.:  ROBOT_IP=192.168.123.161 scripts/run_nav.sh test_map_1.yaml" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  echo "Usage: scripts/waypoint.sh <capture|list|show|run|remove|clear> [args]" >&2
  exit 1
fi

# Run in the SAME container as Nav2; GO2_WAYPOINTS points at the mounted ./maps
# dir so the file survives after the session (container is --rm).
exec docker exec -it go2_robot bash -c '
  source /opt/ros/$ROS_DISTRO/setup.bash
  source /ros2_ws/install/setup.bash
  export GO2_WAYPOINTS="/ros2_ws/maps/'"$WAYPOINTS"'.yaml"
  exec ros2 run go2_robot_sdk waypoint_tool "$@"
' _ "$@"
