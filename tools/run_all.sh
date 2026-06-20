#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2

RUN_DIR="$ROOT/artifacts/runtime"
LOG_DIR="$ROOT/artifacts/runtime/logs"
mkdir -p "$RUN_DIR" "$LOG_DIR"

DASHBOARD_PORT="${DASHBOARD_PORT:-8000}"
ADAS_DASHBOARD_URL="${ADAS_DASHBOARD_URL:-http://localhost:${DASHBOARD_PORT}/api/frame}"
export ADAS_DASHBOARD_URL

COMPONENTS=(dashboard hesai adas rviz)
ENABLED_DASHBOARD=1
ENABLED_HESAI=1
ENABLED_ADAS=1
ENABLED_RVIZ=1
HESAI_MODE="node"

usage() {
  cat <<'EOF'
Usage:
  tools/run_all.sh [start|stop|restart|status|logs] [options]

Commands:
  start      Start dashboard, Hesai LiDAR, ADAS pipeline, and RViz (default)
  stop       Stop processes started by this script
  restart    Stop then start
  status     Show managed process status
  logs       Print log file paths

Options:
  --no-dashboard   Do not start the browser dashboard
  --no-lidar       Do not start Hesai LiDAR
  --no-adas        Do not start ADAS perception/TTC pipeline
  --no-rviz        Do not start RViz
  --hesai-launch   Use ros2 launch instead of direct hesai_ros_driver node
  -h, --help       Show this help

Environment:
  DASHBOARD_PORT       Dashboard port, default 8000
  ADAS_DASHBOARD_URL   ADAS frame POST URL, default http://localhost:$DASHBOARD_PORT/api/frame
EOF
}

pid_file() {
  printf "%s/%s.pid" "$RUN_DIR" "$1"
}

log_file() {
  printf "%s/%s.log" "$LOG_DIR" "$1"
}

is_running() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

component_pid() {
  local file
  file="$(pid_file "$1")"
  [[ -f "$file" ]] && cat "$file"
}

component_enabled() {
  case "$1" in
    dashboard) [[ "$ENABLED_DASHBOARD" -eq 1 ]] ;;
    hesai) [[ "$ENABLED_HESAI" -eq 1 ]] ;;
    adas) [[ "$ENABLED_ADAS" -eq 1 ]] ;;
    rviz) [[ "$ENABLED_RVIZ" -eq 1 ]] ;;
    *) return 1 ;;
  esac
}

command_for_component() {
  case "$1" in
    dashboard)
      printf "%s\n" ".venv/bin/python app/visualization/web_dashboard/web_server.py ${DASHBOARD_PORT}"
      ;;
    hesai)
      if [[ "$HESAI_MODE" == "launch" ]]; then
        printf "%s\n" "bash tools/run_hesai_launch.sh"
      else
        printf "%s\n" "bash tools/run_hesai_lidar.sh"
      fi
      ;;
    adas)
      printf "%s\n" "bash tools/run_live_ego_comp.sh"
      ;;
    rviz)
      printf "%s\n" "bash tools/run_adas_rviz.sh"
      ;;
    *)
      return 1
      ;;
  esac
}

start_component() {
  local name="$1"
  local pid cmd log

  pid="$(component_pid "$name" || true)"
  if is_running "$pid"; then
    echo "SKIP $name already running pid=$pid"
    return 0
  fi

  cmd="$(command_for_component "$name")" || return 1
  log="$(log_file "$name")"
  : > "$log"
  echo "START $name -> $log"
  setsid bash -lc "$cmd" > "$log" 2>&1 < /dev/null &
  pid="$!"
  echo "$pid" > "$(pid_file "$name")"
}

stop_component() {
  local name="$1"
  local pid file
  file="$(pid_file "$name")"
  pid="$(component_pid "$name" || true)"

  if ! is_running "$pid"; then
    rm -f "$file"
    echo "STOP $name already stopped"
    return 0
  fi

  echo "STOP $name pid=$pid"
  kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! is_running "$pid"; then
      rm -f "$file"
      return 0
    fi
    sleep 0.2
  done
  echo "KILL $name pid=$pid"
  kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  rm -f "$file"
}

start_all() {
  local name
  for name in "${COMPONENTS[@]}"; do
    component_enabled "$name" || continue
    start_component "$name"
    if [[ "$name" == "dashboard" || "$name" == "hesai" ]]; then
      sleep 1
    fi
  done
  status_all
  echo "Dashboard: http://localhost:${DASHBOARD_PORT}"
}

stop_all() {
  local index name
  for ((index=${#COMPONENTS[@]}-1; index>=0; index--)); do
    name="${COMPONENTS[$index]}"
    component_enabled "$name" || continue
    stop_component "$name"
  done
}

status_all() {
  local name pid status cmd
  for name in "${COMPONENTS[@]}"; do
    component_enabled "$name" || continue
    pid="$(component_pid "$name" || true)"
    if is_running "$pid"; then
      status="running"
      cmd="$(ps -p "$pid" -o cmd= 2>/dev/null || true)"
    else
      status="stopped"
      cmd=""
    fi
    printf "%-10s %-8s pid=%-8s log=%s %s\n" "$name" "$status" "${pid:-"-"}" "$(log_file "$name")" "$cmd"
  done
}

logs_all() {
  local name
  for name in "${COMPONENTS[@]}"; do
    component_enabled "$name" || continue
    printf "%-10s %s\n" "$name" "$(log_file "$name")"
  done
}

COMMAND="start"
if [[ "${1:-}" =~ ^(start|stop|restart|status|logs)$ ]]; then
  COMMAND="$1"
  shift
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-dashboard) ENABLED_DASHBOARD=0 ;;
    --no-lidar) ENABLED_HESAI=0 ;;
    --no-adas) ENABLED_ADAS=0 ;;
    --no-rviz) ENABLED_RVIZ=0 ;;
    --hesai-launch) HESAI_MODE="launch" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

case "$COMMAND" in
  start) start_all ;;
  stop) stop_all ;;
  restart) stop_all; start_all ;;
  status) status_all ;;
  logs) logs_all ;;
  *) usage; exit 2 ;;
esac
