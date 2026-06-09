#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

ros2 launch hesai_ros_driver start.py
status=$?
echo "HESAI_LAUNCH_EXIT status=$status"
exit "$status"
