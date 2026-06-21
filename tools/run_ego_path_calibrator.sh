#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"

export PYTHONPATH="$ROOT/openpilot:$ROOT/openpilot/opendbc_repo:${PYTHONPATH:-}"

python3 tools/ego_path_calibrator.py "$@"
status=$?
echo "EGO_PATH_CALIBRATOR_EXIT status=$status"
exit "$status"
