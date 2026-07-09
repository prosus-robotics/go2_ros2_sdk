#!/usr/bin/env bash
#
# Mapping + Navigation: slam_toolbox builds the map live while Nav2 plans and
# drives in it (go2_robot_sdk/robot.launch.py with slam:=true nav2:=true).
# Good for exploring a new space and sending goals at the same time. You can
# still snapshot the map mid-run with scripts/save_map.sh <name>.
#
# Usage:
#     ROBOT_IP=192.168.8.181 scripts/run_mapping_nav.sh
#     ROBOT_IP=... scripts/run_mapping_nav.sh ekf:=true rviz2:=false
#
# Extra args are forwarded (e.g. ekf:=true, rviz2:=false, foxglove:=true).
#
set -euo pipefail
cd "$(dirname "$0")/.."

: "${ROBOT_IP:?Set ROBOT_IP=<the Go2 IP>, e.g. ROBOT_IP=192.168.8.181 scripts/run_mapping_nav.sh}"

# Guard against the common mixup: this script BUILDS a map (SLAM); it can't LOAD
# one. A bare *.yaml arg would be passed to the launch and rejected as malformed.
for arg in "$@"; do
  if [[ "$arg" == *.yaml && "$arg" != *:=* ]]; then
    echo "'$arg' looks like a saved map, but run_mapping_nav.sh builds a NEW map." >&2
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
  unitree_ros ros2 launch go2_robot_sdk robot.launch.py slam:=true nav2:=true "$@"
