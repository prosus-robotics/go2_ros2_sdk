#!/usr/bin/env bash
#
# Mapping mode: drive the Go2 around to build a 2D SLAM map (slam_toolbox).
# Teleop + SLAM only, no Nav2. When the map looks good, save it from a SECOND
# terminal WITHOUT stopping this one:
#     scripts/save_map.sh my_map
#
# Usage:
#     ROBOT_IP=192.168.8.181 scripts/run_mapping.sh
#     ROBOT_IP=... MAP_NAME=office scripts/run_mapping.sh rviz2:=false
#
# Extra args are forwarded to the launch (e.g. rviz2:=false, foxglove:=true).
#
set -euo pipefail
cd "$(dirname "$0")/.."

: "${ROBOT_IP:?Set ROBOT_IP=<the Go2 IP>, e.g. ROBOT_IP=192.168.8.181 scripts/run_mapping.sh}"

# Guard against the common mixup: this script BUILDS a map (SLAM); it can't LOAD
# one. A bare *.yaml arg would be passed to the launch and rejected as malformed.
for arg in "$@"; do
  if [[ "$arg" == *.yaml && "$arg" != *:=* ]]; then
    echo "'$arg' looks like a saved map, but run_mapping.sh builds a NEW map." >&2
    echo "To navigate a SAVED map, use run_nav.sh instead:" >&2
    echo "    ROBOT_IP=$ROBOT_IP scripts/run_nav.sh $arg" >&2
    exit 1
  fi
done

mkdir -p maps
docker rm -f go2_robot >/dev/null 2>&1 || true

exec docker compose -f docker/docker-compose.yml run --rm --name go2_robot \
  -e ROBOT_IP="$ROBOT_IP" \
  -e MAP_NAME="${MAP_NAME:-my_map}" \
  -e MAP_SAVE="${MAP_SAVE:-true}" \
  -v "$(pwd)/maps:/ros2_ws/maps" \
  unitree_ros ros2 launch go2_robot_sdk mapping.launch.py "$@"
