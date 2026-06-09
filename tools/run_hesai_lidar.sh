#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

CONFIG_PATH="$ROOT/ros2/src/HesaiLidar_ROS_2.0/config/config.yaml"

ros2 run hesai_ros_driver hesai_ros_driver_node --ros-args -p "config_path:=$CONFIG_PATH"
status=$?
echo "HESAI_EXIT status=$status"
exit "$status"
