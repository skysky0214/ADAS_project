#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="${1:-$ROOT/rosbags/live_full_${STAMP}}"
if [[ $# -gt 0 ]]; then
  shift
fi

mkdir -p "$ROOT/rosbags"

TOPICS=(
  /lidar_points
  /adas/tracking_markers
  /vehicle/ego_motion
  /vehicle/can/raw
  /tf
  /tf_static
)

echo "Recording rosbag:"
echo "  output: $OUTPUT"
echo "  topics: ${TOPICS[*]}"

ros2 bag record \
  -s sqlite3 \
  -o "$OUTPUT" \
  "${TOPICS[@]}" \
  "$@"
status=$?
echo "BAG_RECORD_EXIT status=$status output=$OUTPUT"
exit "$status"
