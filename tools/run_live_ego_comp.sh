#!/usr/bin/env bash
set +u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

source /opt/ros/humble/setup.bash
source "$ROOT/ros2/install/setup.bash"
source "$ROOT/.venv/bin/activate"

export OPENPCDET_ROOT="${OPENPCDET_ROOT:-$ROOT/../OpenPCDet}"
export PYTHONPATH="$OPENPCDET_ROOT:${PYTHONPATH:-}"

PEDESTRIAN_MIN_POINT_MAX_DISTANCE="${PEDESTRIAN_MIN_POINT_MAX_DISTANCE:-}"
PEDESTRIAN_MAX_POINT_MAX_DISTANCE="${PEDESTRIAN_MAX_POINT_MAX_DISTANCE:-}"
EGO_STEER_BIAS_DEG="${EGO_STEER_BIAS_DEG:-2.0}"
PEDESTRIAN_POINT_SPREAD_ARGS=()
if [ -n "$PEDESTRIAN_MIN_POINT_MAX_DISTANCE" ]; then
  PEDESTRIAN_POINT_SPREAD_ARGS+=(--pedestrian-min-point-max-distance "$PEDESTRIAN_MIN_POINT_MAX_DISTANCE")
fi
if [ -n "$PEDESTRIAN_MAX_POINT_MAX_DISTANCE" ]; then
  PEDESTRIAN_POINT_SPREAD_ARGS+=(--pedestrian-max-point-max-distance "$PEDESTRIAN_MAX_POINT_MAX_DISTANCE")
fi

if [ -d "$ROOT/.cuda-nvcc-12.4" ]; then
  export CUDA_HOME="$ROOT/.cuda-nvcc-12.4"
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib:$CUDA_HOME/targets/x86_64-linux/lib:${LD_LIBRARY_PATH:-}"
fi

TORCH_LIB="$(python - <<'PY'
from pathlib import Path
import torch
print(Path(torch.__file__).resolve().parent / "lib")
PY
)"
export LD_LIBRARY_PATH="$TORCH_LIB:${LD_LIBRARY_PATH:-}"

python app/main.py \
  --topic /lidar_points \
  --perception pointpillar \
  --score-threshold 0.29 \
  "${PEDESTRIAN_POINT_SPREAD_ARGS[@]}" \
  --prediction srlstm \
  --prediction-fps 2.5 \
  --latency-playback-rate 1.0 \
  --ego-compensation \
  --ego-can-bus 0 \
  --ego-steer-bias-deg "$EGO_STEER_BIAS_DEG" \
  --safety-radius 1.0 \
  --vehicle-front 2.40 \
  --vehicle-rear 2.10 \
  --vehicle-side 1.00 \
  --marker-frame hesai_lidar \
  --dashboard-url "${ADAS_DASHBOARD_URL:-http://localhost:8000/api/frame}" \
  --output-dir artifacts/live_ego_comp \
  --print-every 20 \
  "$@"
status=$?
echo "ADAS_EXIT status=$status"
exit "$status"
