#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

rviz2 -d "$ROOT/artifacts/rviz/adas_lidar_tracking.rviz"
status=$?
echo "RVIZ_EXIT status=$status"
exit "$status"
