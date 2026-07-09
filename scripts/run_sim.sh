#!/usr/bin/env bash
#
# Run the Tier-0 Nav2 sim (go2_nav_sim) in Docker. Everything runs inside the
# one Linux container, so this works on macOS - view it in Foxglove Studio at
# ws://localhost:8765 (open Foxglove -> Open connection -> that URL).
#
# This is the SAME Nav2 stack + params the real robot uses, driven by a
# kinematic fake base, so it's how you sanity-check nav behaviour and replay a
# real map before/after a hardware session.
#
# Usage:
#     scripts/run_sim.sh                                # built-in example world @ (2,2)
#     scripts/run_sim.sh maps/my_map.yaml               # your saved map @ (0,0)
#     scripts/run_sim.sh maps/my_map.yaml 1.5 -0.5 1.57 # + explicit start x y yaw
#
# The map path is relative to the repo root (the whole repo is mounted into the
# container). maps/<name>.yaml is exactly what scripts/save_map.sh writes.
#
set -euo pipefail
cd "$(dirname "$0")/.."

MAP_ARG="${1:-example}"
START_X="${2:-}"
START_Y="${3:-}"
START_YAW="${4:-0.0}"

if [[ "$MAP_ARG" == "example" || -z "$MAP_ARG" ]]; then
  # Empty SIM_MAP -> the entrypoint generates + loads the example world.
  export SIM_MAP=""
  : "${START_X:=2.0}"   # example world's free space is around (2,2), not (0,0)
  : "${START_Y:=2.0}"
else
  rel="${MAP_ARG#./}"
  if [[ ! -f "$rel" ]]; then
    echo "Map '$rel' not found in the repo." >&2
    echo "Save one from a mapping run first:  scripts/save_map.sh <name>" >&2
    exit 1
  fi
  # Repo root is mounted at /ros2_ws/src, so translate the host path.
  export SIM_MAP="/ros2_ws/src/$rel"
  : "${START_X:=0.0}"   # saved maps are origin-referenced to where you started SLAM
  : "${START_Y:=0.0}"
fi

export SIM_START_X="$START_X" SIM_START_Y="$START_Y" SIM_START_YAW="$START_YAW"

echo "Sim map:   ${SIM_MAP:-<generated example world>}"
echo "Start pose: ($SIM_START_X, $SIM_START_Y, yaw=$SIM_START_YAW)"
echo "Foxglove:  ws://localhost:8765   (Ctrl+C here to stop)"
echo ""
exec docker compose -f docker/docker-compose.yml up sim
