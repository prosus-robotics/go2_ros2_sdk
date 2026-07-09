#!/usr/bin/env bash
#
# Navigation on a SAVED map: AMCL localization + Nav2, no SLAM
# (go2_robot_sdk/navigation.launch.py). The map (.yaml + .pgm) must live in
# ./maps - that's exactly where scripts/save_map.sh writes it.
#
# Usage:
#     ROBOT_IP=192.168.8.181 scripts/run_nav.sh my_map.yaml
#     ROBOT_IP=... scripts/run_nav.sh office.yaml rviz2:=false
#
# LOCALIZATION IS SEAMLESS BY DEFAULT: the launch auto-publishes an initial pose
# of (0,0,0) a few seconds after start, so if you place the robot back at the
# SAME spot you started mapping from ("dock"), AMCL just locks on - no clicking.
# If you start somewhere else, tell it where:
#     ROBOT_IP=... INIT_X=1.5 INIT_Y=-0.5 INIT_YAW=1.57 scripts/run_nav.sh my_map.yaml
# You can also always correct it live via RViz "2D Pose Estimate" / Foxglove.
#
set -euo pipefail
cd "$(dirname "$0")/.."

: "${ROBOT_IP:?Set ROBOT_IP=<the Go2 IP>, e.g. ROBOT_IP=192.168.8.181 scripts/run_nav.sh my_map.yaml}"
MAP_YAML="${1:?Usage: scripts/run_nav.sh <map.yaml in ./maps>  e.g. scripts/run_nav.sh my_map.yaml}"
shift
MAP_BASE="$(basename "$MAP_YAML")"
if [[ ! -f "maps/$MAP_BASE" ]]; then
  echo "maps/$MAP_BASE not found. Save one first:  scripts/save_map.sh <name>" >&2
  exit 1
fi
docker rm -f go2_robot >/dev/null 2>&1 || true

exec docker compose -f docker/docker-compose.yml run --rm --name go2_robot \
  -e ROBOT_IP="$ROBOT_IP" \
  -e INIT_X="${INIT_X:-0.0}" \
  -e INIT_Y="${INIT_Y:-0.0}" \
  -e INIT_YAW="${INIT_YAW:-0.0}" \
  -v "$(pwd)/maps:/ros2_ws/maps" \
  unitree_ros ros2 launch go2_robot_sdk navigation.launch.py \
    map:="/ros2_ws/maps/$MAP_BASE" "$@"
