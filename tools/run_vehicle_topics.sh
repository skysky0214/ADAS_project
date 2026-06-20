#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"
source "$ROOT/.venv/bin/activate"

export PYTHONPATH="$ROOT/openpilot:${PYTHONPATH:-}"

python tools/publish_vehicle_topics.py \
  --bus "${EGO_CAN_BUS:-0}" \
  --can-speed "${EGO_CAN_SPEED:-500}" \
  --data-speed "${EGO_CAN_DATA_SPEED:-2000}" \
  "$@"
status=$?
echo "VEHICLE_TOPICS_EXIT status=$status"
exit "$status"
