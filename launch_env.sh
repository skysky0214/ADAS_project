#!/usr/bin/env bash

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -f "$PROJECT_ROOT/ros2/install/setup.bash" ]; then
  source "$PROJECT_ROOT/ros2/install/setup.bash"
fi

source "$PROJECT_ROOT/.venv/bin/activate"

export MPLCONFIGDIR="$PROJECT_ROOT/.cache/matplotlib"
mkdir -p "$MPLCONFIGDIR"

if [ -d "$PROJECT_ROOT/../OpenPCDet" ]; then
  export OPENPCDET_ROOT="$PROJECT_ROOT/../OpenPCDet"
  export PYTHONPATH="$OPENPCDET_ROOT:${PYTHONPATH:-}"
fi

if [ -d "$PROJECT_ROOT/.cuda-nvcc-12.4" ]; then
  export CUDA_HOME="$PROJECT_ROOT/.cuda-nvcc-12.4"
  export PATH="$CUDA_HOME/bin:$PATH"
  export LD_LIBRARY_PATH="$CUDA_HOME/lib:$CUDA_HOME/targets/x86_64-linux/lib:${LD_LIBRARY_PATH:-}"
  export LIBRARY_PATH="$CUDA_HOME/lib:$CUDA_HOME/targets/x86_64-linux/lib:${LIBRARY_PATH:-}"
fi

TORCH_LIB="$(python - <<'PY'
from pathlib import Path
import torch
print(Path(torch.__file__).resolve().parent / "lib")
PY
)"
export LD_LIBRARY_PATH="$TORCH_LIB:${LD_LIBRARY_PATH:-}"

echo "ADAS main environment ready: $PROJECT_ROOT"
echo "python: $(command -v python)"
python - <<'PY'
import open3d
import torch
print(f"open3d: {open3d.__version__}")
print(f"torch: {torch.__version__}")
PY
