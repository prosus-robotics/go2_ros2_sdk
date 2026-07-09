#!/usr/bin/env bash
#
# Rebuild the robot Docker image so the latest code / config / launch changes are
# baked in. REQUIRED after editing anything under go2_robot_sdk/ (nav2_params,
# twist_mux, ekf, the launch files, waypoint_tool, ...) because the `unitree_ros`
# service COMPILES the source INTO the image at build time - it does not
# bind-mount the source live (unlike the `sim` service). Without a rebuild the
# robot keeps running the OLD config.
#
# Usage:
#     scripts/build_robot_image.sh
#
set -euo pipefail
cd "$(dirname "$0")/.."

echo "Rebuilding unitree_ros image (colcon build inside Docker, ~a few minutes)..."
exec docker compose -f docker/docker-compose.yml build unitree_ros
