#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO


ROOT = Path(__file__).resolve().parents[1]
OPENPILOT_ROOT = ROOT / "openpilot"

import rclpy  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402


DEFAULT_STEER_RATIO = 14.25


@dataclass
class ControlState:
    steer_ratio: float
    wheelbase: float
    invert_steer: bool


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class EgoDelta:
    dx_m: float = 0.0
    dy_m: float = 0.0
    dyaw_rad: float = 0.0
    speed_mps: float = 0.0
    wheel_avg_kph: float = 0.0
    steering_deg: float = 0.0
    road_angle_rad: float = 0.0
    yaw_rate_radps: float = 0.0
    wheel_speeds_kph: list[float] | None = None
    valid: bool = False
    reset: bool = False


class TerminalControls:
    def __init__(self, enabled: bool):
        self.enabled = enabled and sys.stdin.isatty()
        self._fd: int | None = None
        self._old_attrs = None

    def __enter__(self) -> "TerminalControls":
        if self.enabled:
            self._fd = sys.stdin.fileno()
            self._old_attrs = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self.enabled and self._fd is not None and self._old_attrs is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)

    def read_key(self) -> str | None:
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)


class PandaEgoReaderProcess:
    def __init__(self, args: argparse.Namespace, controls: ControlState):
        self.args = args
        self.controls = controls
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._error: str | None = None
        self._pending_dx = 0.0
        self._pending_dy = 0.0
        self._pending_dyaw = 0.0
        self._latest = EgoDelta(wheel_speeds_kph=[0.0, 0.0, 0.0, 0.0])

        python = OPENPILOT_ROOT / ".venv" / "bin" / "python"
        if not python.exists():
            raise FileNotFoundError(f"openpilot Python not found: {python}")

        cmd = [
            str(python),
            "-u",
            str(ROOT / "tools" / "panda_ego_reader.py"),
            "--bus",
            str(args.bus),
            "--can-speed",
            str(args.can_speed),
            "--data-speed",
            str(args.data_speed),
            "--wheelbase",
            str(controls.wheelbase),
            "--steer-ratio",
            str(controls.steer_ratio),
            "--angle-source",
            args.angle_source,
            "--max-dt",
            str(args.max_dt),
            "--emit-hz",
            str(args.ego_emit_hz),
        ]
        if args.no_config:
            cmd.append("--no-config")
        if controls.invert_steer:
            cmd.append("--invert-steer")

        env = os.environ.copy()
        env["PYTHONPATH"] = f"{OPENPILOT_ROOT}:{OPENPILOT_ROOT / 'opendbc_repo'}:{env.get('PYTHONPATH', '')}"
        self._proc = subprocess.Popen(
            cmd,
            cwd=ROOT,
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.update_controls(controls)

    @property
    def error(self) -> str | None:
        return self._error

    def wait_ready(self, timeout_sec: float) -> bool:
        return self._ready.wait(timeout_sec)

    def update_controls(self, controls: ControlState) -> None:
        self.controls = controls
        if self._proc.stdin is None or self._proc.poll() is not None:
            return
        payload = {
            "steer_ratio": controls.steer_ratio,
            "wheelbase": controls.wheelbase,
            "invert_steer": controls.invert_steer,
        }
        try:
            self._proc.stdin.write("CONTROL " + json.dumps(payload, separators=(",", ":")) + "\n")
            self._proc.stdin.flush()
        except BrokenPipeError:
            self._error = "panda ego reader pipe closed"

    def pop_delta(self) -> EgoDelta:
        with self._lock:
            delta = EgoDelta(
                dx_m=self._pending_dx,
                dy_m=self._pending_dy,
                dyaw_rad=self._pending_dyaw,
                speed_mps=self._latest.speed_mps,
                wheel_avg_kph=self._latest.wheel_avg_kph,
                steering_deg=self._latest.steering_deg,
                road_angle_rad=self._latest.road_angle_rad,
                yaw_rate_radps=self._latest.yaw_rate_radps,
                wheel_speeds_kph=list(self._latest.wheel_speeds_kph or [0.0, 0.0, 0.0, 0.0]),
                valid=self._latest.valid,
                reset=self._latest.reset,
            )
            self._pending_dx = 0.0
            self._pending_dy = 0.0
            self._pending_dyaw = 0.0
            self._latest.reset = False
            return delta

    def stop(self) -> None:
        self._stop.set()
        if self._proc.poll() is None:
            self._proc.terminate()
        self._thread.join(timeout=2.0)
        if self._proc.poll() is None:
            self._proc.kill()

    def _read_loop(self) -> None:
        assert self._proc.stdout is not None
        for raw_line in self._proc.stdout:
            if self._stop.is_set():
                break
            line = raw_line.strip()
            if line == "EGO_READY":
                self._ready.set()
                continue
            if line.startswith("EGO_ERROR "):
                self._error = line.removeprefix("EGO_ERROR ")
                self._ready.set()
                break
            if line.startswith("EGO "):
                try:
                    self._apply_ego(json.loads(line[4:]))
                except json.JSONDecodeError:
                    continue
                continue
            if line:
                print(f"[ego-reader] {line}", flush=True)
        if not self._ready.is_set():
            self._error = "panda ego reader exited before ready"
            self._ready.set()
        if self._proc.poll() not in (None, 0) and self._error is None:
            self._error = f"panda ego reader exited with code {self._proc.returncode}"

    def _apply_ego(self, data: dict) -> None:
        with self._lock:
            if not data.get("reset", False):
                self._pending_dx += float(data.get("dx_m", 0.0))
                self._pending_dy += float(data.get("dy_m", 0.0))
                self._pending_dyaw += float(data.get("dyaw_rad", 0.0))
            self._latest = EgoDelta(
                speed_mps=float(data.get("speed_mps", 0.0)),
                wheel_avg_kph=float(data.get("wheel_avg_kph", 0.0)),
                steering_deg=float(data.get("steering_deg", 0.0)),
                road_angle_rad=float(data.get("road_angle_rad", 0.0)),
                yaw_rate_radps=float(data.get("yaw_rate_radps", 0.0)),
                wheel_speeds_kph=list(data.get("wheel_speeds_kph", [0.0, 0.0, 0.0, 0.0])),
                valid=bool(data.get("valid", False)),
                reset=bool(data.get("reset", False)),
            )


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def set_yaw(marker: Marker, yaw: float) -> None:
    marker.pose.orientation.z = math.sin(yaw * 0.5)
    marker.pose.orientation.w = math.cos(yaw * 0.5)


def base_marker(frame_id: str, namespace: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.lifetime = Duration(sec=0, nanosec=0)
    return marker


def rear_axle_to_center(
    rear_pose: Pose2D,
    wheelbase: float,
    front_overhang: float,
    rear_overhang: float,
) -> tuple[float, float]:
    offset = ((front_overhang + wheelbase + rear_overhang) * 0.5) - rear_overhang
    return (
        rear_pose.x + offset * math.cos(rear_pose.yaw),
        rear_pose.y + offset * math.sin(rear_pose.yaw),
    )


def read_control_file(path: Path, current: ControlState) -> tuple[ControlState, bool]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return current, False
    except json.JSONDecodeError as exc:
        print(f"[control] ignoring invalid JSON: {exc}", flush=True)
        return current, False

    reset_requested = bool(data.get("reset_pose", False))
    next_state = ControlState(
        steer_ratio=float(data.get("steer_ratio", current.steer_ratio)),
        wheelbase=float(data.get("wheelbase", current.wheelbase)),
        invert_steer=bool(data.get("invert_steer", current.invert_steer)),
    )
    return next_state, reset_requested


def write_control_file(path: Path, controls: ControlState, reset_pose: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "steer_ratio": round(controls.steer_ratio, 6),
        "wheelbase": round(controls.wheelbase, 6),
        "invert_steer": controls.invert_steer,
        "reset_pose": reset_pose,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def integrate_delta(pose: Pose2D, delta: EgoDelta) -> None:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    pose.x += (cos_yaw * delta.dx_m) - (sin_yaw * delta.dy_m)
    pose.y += (sin_yaw * delta.dx_m) + (cos_yaw * delta.dy_m)
    pose.yaw = wrap_pi(pose.yaw + delta.dyaw_rad)


def append_path_point(
    rear_path: list[tuple[float, float]],
    center_path: list[tuple[float, float]],
    rear_pose: Pose2D,
    center_xy: tuple[float, float],
    min_step_m: float,
    max_points: int,
    force: bool = False,
) -> None:
    if force or not rear_path:
        rear_path.append((rear_pose.x, rear_pose.y))
        center_path.append(center_xy)
    else:
        last_x, last_y = rear_path[-1]
        if math.hypot(rear_pose.x - last_x, rear_pose.y - last_y) >= min_step_m:
            rear_path.append((rear_pose.x, rear_pose.y))
            center_path.append(center_xy)

    if len(rear_path) > max_points:
        del rear_path[: len(rear_path) - max_points]
    if len(center_path) > max_points:
        del center_path[: len(center_path) - max_points]


def reset_pose(
    rear_pose: Pose2D,
    rear_path: list[tuple[float, float]],
    center_path: list[tuple[float, float]],
    controls: ControlState,
    args: argparse.Namespace,
) -> None:
    rear_pose.x = 0.0
    rear_pose.y = 0.0
    rear_pose.yaw = 0.0
    rear_path.clear()
    center_path.clear()
    center_xy = rear_axle_to_center(rear_pose, controls.wheelbase, args.front_overhang, args.rear_overhang)
    append_path_point(rear_path, center_path, rear_pose, center_xy, args.path_step_m, args.max_path_points, force=True)


def build_markers(
    frame_id: str,
    rear_pose: Pose2D,
    rear_path: list[tuple[float, float]],
    center_path: list[tuple[float, float]],
    controls: ControlState,
    sample: EgoDelta,
    distance_m: float,
    args: argparse.Namespace,
) -> MarkerArray:
    markers = MarkerArray()

    delete = Marker()
    delete.header.frame_id = frame_id
    delete.action = Marker.DELETEALL
    markers.markers.append(delete)

    center_x, center_y = rear_axle_to_center(rear_pose, controls.wheelbase, args.front_overhang, args.rear_overhang)
    start_x, start_y = center_path[0] if center_path else (center_x, center_y)
    drift_x = center_x - start_x
    drift_y = center_y - start_y
    drift_norm = math.hypot(drift_x, drift_y)

    center_line = base_marker(frame_id, "ego_center_path", 1, Marker.LINE_STRIP)
    center_line.scale.x = 0.16
    center_line.points = [point(x, y, args.path_z) for x, y in center_path]
    set_color(center_line, (0.0, 0.85, 1.0, 1.0))
    markers.markers.append(center_line)

    rear_line = base_marker(frame_id, "ego_rear_axle_path", 1, Marker.LINE_STRIP)
    rear_line.scale.x = 0.08
    rear_line.points = [point(x, y, args.path_z + 0.08) for x, y in rear_path]
    set_color(rear_line, (1.0, 0.85, 0.05, 0.85))
    markers.markers.append(rear_line)

    closure = base_marker(frame_id, "closure_vector", 1, Marker.LINE_STRIP)
    closure.scale.x = 0.06
    closure.points = [point(start_x, start_y, args.path_z + 0.16), point(center_x, center_y, args.path_z + 0.16)]
    set_color(closure, (1.0, 0.1, 1.0, 0.85))
    markers.markers.append(closure)

    start = base_marker(frame_id, "start_pose", 1, Marker.SPHERE)
    start.pose.position.x = float(start_x)
    start.pose.position.y = float(start_y)
    start.pose.position.z = args.path_z + 0.25
    start.scale.x = start.scale.y = start.scale.z = 0.55
    set_color(start, (0.0, 1.0, 0.25, 1.0))
    markers.markers.append(start)

    vehicle = base_marker(frame_id, "ego_vehicle", 1, Marker.CUBE)
    vehicle.pose.position.x = float(center_x)
    vehicle.pose.position.y = float(center_y)
    vehicle.pose.position.z = args.vehicle_height * 0.5
    set_yaw(vehicle, rear_pose.yaw)
    vehicle.scale.x = args.vehicle_length
    vehicle.scale.y = args.vehicle_width
    vehicle.scale.z = args.vehicle_height
    set_color(vehicle, (0.1, 0.65, 1.0, 0.42))
    markers.markers.append(vehicle)

    heading = base_marker(frame_id, "ego_heading", 1, Marker.ARROW)
    heading.pose.position.x = float(center_x)
    heading.pose.position.y = float(center_y)
    heading.pose.position.z = args.vehicle_height + 0.2
    set_yaw(heading, rear_pose.yaw)
    heading.scale.x = 2.8
    heading.scale.y = 0.22
    heading.scale.z = 0.22
    set_color(heading, (0.0, 1.0, 0.25, 0.95))
    markers.markers.append(heading)

    status = base_marker(frame_id, "ego_status", 1, Marker.TEXT_VIEW_FACING)
    status.pose.position.x = float(center_x)
    status.pose.position.y = float(center_y)
    status.pose.position.z = args.vehicle_height + 1.3
    status.scale.z = 0.75
    status.text = (
        f"ratio={controls.steer_ratio:.3f} wb={controls.wheelbase:.3f} inv={int(controls.invert_steer)}\n"
        f"v={sample.speed_mps:.2f}m/s steer={sample.steering_deg:.1f}deg "
        f"road={math.degrees(sample.road_angle_rad):.2f}deg\n"
        f"pose=({center_x:.2f},{center_y:.2f}) yaw={math.degrees(rear_pose.yaw):.1f}deg "
        f"dist={distance_m:.1f}m\n"
        f"start_drift=({drift_x:.2f},{drift_y:.2f}) |d|={drift_norm:.2f}m"
    )
    set_color(status, (1.0, 1.0, 1.0, 1.0))
    markers.markers.append(status)

    return markers


def open_log(path: Path | None) -> tuple[TextIO | None, csv.DictWriter | None]:
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    f = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "t_sec",
            "x_rear_m",
            "y_rear_m",
            "x_center_m",
            "y_center_m",
            "yaw_deg",
            "distance_m",
            "speed_mps",
            "wheel_avg_kph",
            "steering_deg",
            "road_angle_deg",
            "yaw_rate_dps",
            "steer_ratio",
            "wheelbase_m",
            "invert_steer",
            "w1_kph",
            "w2_kph",
            "w3_kph",
            "w4_kph",
            "valid",
        ],
    )
    writer.writeheader()
    return f, writer


def write_log_row(
    writer: csv.DictWriter | None,
    start_t: float,
    rear_pose: Pose2D,
    controls: ControlState,
    sample: EgoDelta,
    distance_m: float,
    args: argparse.Namespace,
) -> None:
    if writer is None:
        return
    now = time.monotonic()
    center_x, center_y = rear_axle_to_center(rear_pose, controls.wheelbase, args.front_overhang, args.rear_overhang)
    writer.writerow(
        {
            "t_sec": now - start_t,
            "x_rear_m": rear_pose.x,
            "y_rear_m": rear_pose.y,
            "x_center_m": center_x,
            "y_center_m": center_y,
            "yaw_deg": math.degrees(rear_pose.yaw),
            "distance_m": distance_m,
            "speed_mps": sample.speed_mps,
            "wheel_avg_kph": sample.wheel_avg_kph,
            "steering_deg": sample.steering_deg,
            "road_angle_deg": math.degrees(sample.road_angle_rad),
            "yaw_rate_dps": math.degrees(sample.yaw_rate_radps),
            "steer_ratio": controls.steer_ratio,
            "wheelbase_m": controls.wheelbase,
            "invert_steer": controls.invert_steer,
            "w1_kph": (sample.wheel_speeds_kph or [0.0, 0.0, 0.0, 0.0])[0],
            "w2_kph": (sample.wheel_speeds_kph or [0.0, 0.0, 0.0, 0.0])[1],
            "w3_kph": (sample.wheel_speeds_kph or [0.0, 0.0, 0.0, 0.0])[2],
            "w4_kph": (sample.wheel_speeds_kph or [0.0, 0.0, 0.0, 0.0])[3],
            "valid": sample.valid,
        }
    )


def handle_key(
    key: str,
    controls: ControlState,
    args: argparse.Namespace,
    rear_pose: Pose2D,
    rear_path: list[tuple[float, float]],
    center_path: list[tuple[float, float]],
) -> bool:
    if key in ("q", "\x03"):
        return True
    if key in ("+", "="):
        controls.steer_ratio += args.ratio_step
        print(f"[control] steer_ratio -> {controls.steer_ratio:.3f}", flush=True)
    elif key in ("-", "_"):
        controls.steer_ratio = max(args.min_steer_ratio, controls.steer_ratio - args.ratio_step)
        print(f"[control] steer_ratio -> {controls.steer_ratio:.3f}", flush=True)
    elif key == "]":
        controls.steer_ratio += args.ratio_big_step
        print(f"[control] steer_ratio -> {controls.steer_ratio:.3f}", flush=True)
    elif key == "[":
        controls.steer_ratio = max(args.min_steer_ratio, controls.steer_ratio - args.ratio_big_step)
        print(f"[control] steer_ratio -> {controls.steer_ratio:.3f}", flush=True)
    elif key == "i":
        controls.invert_steer = not controls.invert_steer
        print(f"[control] invert_steer -> {controls.invert_steer}", flush=True)
    elif key == "r":
        reset_pose(rear_pose, rear_path, center_path, controls, args)
        print("[control] reset pose/path", flush=True)
    else:
        return False
    write_control_file(args.control_file, controls)
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Trace EV6 ego path from Panda CAN and tune bicycle-model steer ratio live."
    )
    parser.add_argument("--bus", type=int, default=0, help="Panda bus with EV6 ECAN wheel/steering messages")
    parser.add_argument("--can-speed", type=int, default=500)
    parser.add_argument("--data-speed", type=int, default=2000)
    parser.add_argument("--no-config", action="store_true", help="Do not configure Panda CAN-FD speeds")
    parser.add_argument("--angle-source", choices=("sensor", "mdps"), default="sensor")
    parser.add_argument("--invert-steer", action="store_true")
    parser.add_argument("--steer-ratio", type=float, default=DEFAULT_STEER_RATIO)
    parser.add_argument("--min-steer-ratio", type=float, default=4.0)
    parser.add_argument("--ratio-step", type=float, default=0.05)
    parser.add_argument("--ratio-big-step", type=float, default=0.5)
    parser.add_argument("--wheelbase", type=float, default=2.900)
    parser.add_argument("--front-overhang", type=float, default=0.870)
    parser.add_argument("--rear-overhang", type=float, default=0.785)
    parser.add_argument("--vehicle-length", type=float, default=4.695)
    parser.add_argument("--vehicle-width", type=float, default=1.880)
    parser.add_argument("--vehicle-height", type=float, default=0.35)
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--ego-emit-hz", type=float, default=100.0, help="Panda helper ego-delta output rate")
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--path-step-m", type=float, default=0.08)
    parser.add_argument("--max-path-points", type=int, default=12000)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--marker-topic", default="/ego_path_markers")
    parser.add_argument("--path-z", type=float, default=0.05)
    parser.add_argument(
        "--control-file",
        type=Path,
        default=ROOT / "artifacts" / "ego_path_calibration" / "control.json",
        help="JSON file watched for live steer_ratio/wheelbase/invert_steer changes",
    )
    parser.add_argument(
        "--log-csv",
        type=Path,
        default=ROOT / "artifacts" / "ego_path_calibration" / "ego_path.csv",
    )
    parser.add_argument("--no-keyboard", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    controls = ControlState(args.steer_ratio, args.wheelbase, args.invert_steer)
    write_control_file(args.control_file, controls)

    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rclpy.init(args=None)
    node = rclpy.create_node("ego_path_calibrator")
    publisher = node.create_publisher(MarkerArray, args.marker_topic, 10)

    reader: PandaEgoReaderProcess | None = None
    log_file, log_writer = open_log(args.log_csv)
    rear_pose = Pose2D()
    rear_path: list[tuple[float, float]] = []
    center_path: list[tuple[float, float]] = []
    reset_pose(rear_pose, rear_path, center_path, controls, args)

    start_t = time.monotonic()
    last_publish = 0.0
    last_print = 0.0
    last_control_check = 0.0
    last_control_mtime = args.control_file.stat().st_mtime if args.control_file.exists() else 0.0
    distance_m = 0.0
    publish_dt = 1.0 / max(args.publish_hz, 1e-6)
    print_dt = 1.0 / max(args.print_hz, 1e-6)

    try:
        reader = PandaEgoReaderProcess(args, controls)
        if not reader.wait_ready(10.0):
            raise RuntimeError("Timed out waiting for Panda ego reader")
        if reader.error is not None:
            raise RuntimeError(reader.error)

        print("Panda ego reader ready.", flush=True)
        print(f"Publishing RViz markers: {args.marker_topic} frame={args.frame_id}", flush=True)
        print(f"Control file: {args.control_file}", flush=True)
        print("Keys: +/- ratio small, [/ ] ratio big, i invert steer, r reset path, q quit", flush=True)

        with TerminalControls(enabled=not args.no_keyboard) as terminal:
            while not stop and rclpy.ok():
                now = time.monotonic()
                if reader.error is not None:
                    raise RuntimeError(reader.error)

                key = terminal.read_key()
                if key is not None:
                    stop = handle_key(key, controls, args, rear_pose, rear_path, center_path)
                    if key == "r":
                        distance_m = 0.0
                    reader.update_controls(controls)
                    try:
                        last_control_mtime = args.control_file.stat().st_mtime
                    except FileNotFoundError:
                        last_control_mtime = 0.0

                if now - last_control_check >= 0.2:
                    last_control_check = now
                    try:
                        mtime = args.control_file.stat().st_mtime
                    except FileNotFoundError:
                        mtime = 0.0
                    if mtime and mtime != last_control_mtime:
                        last_control_mtime = mtime
                        controls, reset_requested = read_control_file(args.control_file, controls)
                        if reset_requested:
                            reset_pose(rear_pose, rear_path, center_path, controls, args)
                            distance_m = 0.0
                            write_control_file(args.control_file, controls, reset_pose=False)
                            last_control_mtime = args.control_file.stat().st_mtime
                            print("[control] reset from control file", flush=True)
                        reader.update_controls(controls)
                        print(
                            f"[control] loaded ratio={controls.steer_ratio:.3f} "
                            f"wheelbase={controls.wheelbase:.3f} invert={controls.invert_steer}",
                            flush=True,
                        )

                sample = reader.pop_delta()
                integrate_delta(rear_pose, sample)
                distance_m += math.hypot(sample.dx_m, sample.dy_m)
                center_xy = rear_axle_to_center(rear_pose, controls.wheelbase, args.front_overhang, args.rear_overhang)
                append_path_point(
                    rear_path,
                    center_path,
                    rear_pose,
                    center_xy,
                    args.path_step_m,
                    args.max_path_points,
                )
                write_log_row(log_writer, start_t, rear_pose, controls, sample, distance_m, args)

                if now - last_publish >= publish_dt:
                    last_publish = now
                    marker_array = build_markers(
                        frame_id=args.frame_id,
                        rear_pose=rear_pose,
                        rear_path=rear_path,
                        center_path=center_path,
                        controls=controls,
                        sample=sample,
                        distance_m=distance_m,
                        args=args,
                    )
                    stamp = node.get_clock().now().to_msg()
                    for marker in marker_array.markers:
                        marker.header.stamp = stamp
                    publisher.publish(marker_array)

                if now - last_print >= print_dt:
                    last_print = now
                    center_x, center_y = center_xy
                    start_x, start_y = center_path[0]
                    drift = math.hypot(center_x - start_x, center_y - start_y)
                    print(
                        f"ratio={controls.steer_ratio:6.3f} "
                        f"v={sample.speed_mps:5.2f}m/s steer={sample.steering_deg:7.2f}deg "
                        f"yaw={math.degrees(rear_pose.yaw):7.2f}deg "
                        f"center=({center_x:8.2f},{center_y:8.2f}) "
                        f"dist={distance_m:8.1f}m start_drift={drift:6.2f}m",
                        flush=True,
                    )

                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.001)
    finally:
        if log_file is not None:
            log_file.close()
        if reader is not None:
            reader.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("Stopped.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
