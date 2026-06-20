from __future__ import annotations

import math
import json
import os
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path


KPH_TO_MS = 1000.0 / 3600.0
DEFAULT_STEER_RATIO = 14.25
SAFETY_SILENT = 0
ADDR_WHEEL_SPEEDS = 160
ADDR_ACCELERATOR = 53
ADDR_ACCELERATOR_BRAKE_ALT = 256
ADDR_ACCELERATOR_ALT = 261
ADDR_MDPS = 234
ADDR_STEERING_SENSORS = 293
ADDR_TCS = 373


@dataclass(frozen=True)
class EgoMotionDelta:
    dx_m: float = 0.0
    dy_m: float = 0.0
    dyaw_rad: float = 0.0
    speed_mps: float = 0.0
    steering_deg: float = 0.0
    accelerator_pressed: bool = False
    accelerator_pedal: float = 0.0
    accelerator_pedal_raw: int = 0
    brake_pressed: bool = False
    brake_lights: bool = False
    valid: bool = False
    reset: bool = False


@dataclass
class _CanEgoState:
    wheel_speeds_kph: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    moving_backward: bool = False
    steering_sensor_deg: float = 0.0
    mdps_steering_deg: float = 0.0
    accelerator_pressed: bool = False
    accelerator_pedal: float = 0.0
    accelerator_pedal_raw: int = 0
    brake_pressed: bool = False
    brake_lights: bool = False
    seen_wheel_speeds: bool = False
    seen_steering: bool = False


class PandaEgoMotionReader:
    """Read EV6 CAN ego motion and expose frame-to-frame ego deltas.

    The delta is expressed in the previous LiDAR/ego frame:
      - +x is forward
      - +y is left
      - +yaw is left turn / CCW

    Consumers can transform old-frame points into the current frame with:
      p_current = R(-dyaw) @ (p_old - [dx, dy])
    """

    def __init__(
        self,
        bus: int,
        can_speed: int,
        data_speed: int,
        configure_panda: bool,
        wheelbase_m: float,
        steer_ratio: float,
        angle_source: str,
        invert_steer: bool,
        max_dt_sec: float,
        stop_speed_threshold_mps: float,
        stop_reset_sec: float,
    ):
        self.bus = bus
        self.can_speed = can_speed
        self.data_speed = data_speed
        self.configure_panda = configure_panda
        self.wheelbase_m = wheelbase_m
        self.steer_ratio = steer_ratio
        self.angle_source = angle_source
        self.invert_steer = invert_steer
        self.max_dt_sec = max_dt_sec
        self.stop_speed_threshold_mps = stop_speed_threshold_mps
        self.stop_reset_sec = stop_reset_sec

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error: str | None = None
        self._pending_dx_m = 0.0
        self._pending_dy_m = 0.0
        self._pending_dyaw_rad = 0.0
        self._latest_speed_mps = 0.0
        self._latest_steering_deg = 0.0
        self._latest_accelerator_pressed = False
        self._latest_accelerator_pedal = 0.0
        self._latest_accelerator_pedal_raw = 0
        self._latest_brake_pressed = False
        self._latest_brake_lights = False
        self._has_sample = False
        self._reset_pending = False
        self._stopped_since: float | None = None
        self._stopped_reset_sent = False
        self._panda = None
        self._proc: subprocess.Popen | None = None

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @property
    def error(self) -> str | None:
        return self._error

    def wait_ready(self, timeout_sec: float = 2.0) -> bool:
        return self._ready.wait(timeout=timeout_sec)

    def stop(self) -> None:
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
        self._thread.join(timeout=2.0)
        if self._proc is not None and self._proc.poll() is None:
            self._proc.kill()

    def pop_delta(self) -> EgoMotionDelta:
        with self._lock:
            delta = EgoMotionDelta(
                dx_m=self._pending_dx_m,
                dy_m=self._pending_dy_m,
                dyaw_rad=self._pending_dyaw_rad,
                speed_mps=self._latest_speed_mps,
                steering_deg=self._latest_steering_deg,
                accelerator_pressed=self._latest_accelerator_pressed,
                accelerator_pedal=self._latest_accelerator_pedal,
                accelerator_pedal_raw=self._latest_accelerator_pedal_raw,
                brake_pressed=self._latest_brake_pressed,
                brake_lights=self._latest_brake_lights,
                valid=self._has_sample,
                reset=self._reset_pending,
            )
            self._pending_dx_m = 0.0
            self._pending_dy_m = 0.0
            self._pending_dyaw_rad = 0.0
            self._reset_pending = False
            return delta

    def _run(self) -> None:
        try:
            project_root = Path(__file__).resolve().parents[1]
            openpilot_root = project_root / "openpilot"
            helper = project_root / "tools" / "panda_ego_reader.py"
            python = openpilot_root / ".venv" / "bin" / "python"
            if not python.exists():
                python = Path(sys.executable)

            cmd = [
                str(python),
                "-u",
                str(helper),
                "--bus",
                str(self.bus),
                "--can-speed",
                str(self.can_speed),
                "--data-speed",
                str(self.data_speed),
                "--wheelbase",
                str(self.wheelbase_m),
                "--steer-ratio",
                str(self.steer_ratio),
                "--angle-source",
                self.angle_source,
                "--max-dt",
                str(self.max_dt_sec),
                "--stop-speed-threshold",
                str(self.stop_speed_threshold_mps),
                "--stop-reset-sec",
                str(self.stop_reset_sec),
            ]
            if not self.configure_panda:
                cmd.append("--no-config")
            if self.invert_steer:
                cmd.append("--invert-steer")

            env = os.environ.copy()
            env["PYTHONPATH"] = str(openpilot_root)
            self._proc = subprocess.Popen(
                cmd,
                cwd=project_root,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            assert self._proc.stdout is not None
            for raw_line in self._proc.stdout:
                if self._stop.is_set():
                    break
                line = raw_line.strip()
                if line == "EGO_READY":
                    self._ready.set()
                elif line.startswith("EGO_ERROR "):
                    self._error = line.removeprefix("EGO_ERROR ")
                    self._ready.set()
                    break
                elif line.startswith("EGO "):
                    self._apply_subprocess_delta(json.loads(line[4:]))

            if not self._ready.is_set():
                self._error = "Panda ego reader exited before becoming ready"
                self._ready.set()
            if self._proc.poll() not in (None, 0) and self._error is None:
                self._error = f"Panda ego reader exited with code {self._proc.returncode}"
        except Exception as exc:  # pragma: no cover - hardware/runtime path
            self._error = f"{type(exc).__name__}: {exc}"
            self._ready.set()
        finally:
            if self._proc is not None and self._proc.poll() is None:
                self._proc.terminate()

    def _apply_subprocess_delta(self, data: dict) -> None:
        reset = bool(data.get("reset", False))
        with self._lock:
            if reset:
                self._pending_dx_m = 0.0
                self._pending_dy_m = 0.0
                self._pending_dyaw_rad = 0.0
                self._reset_pending = True
            else:
                self._pending_dx_m += float(data.get("dx_m", 0.0))
                self._pending_dy_m += float(data.get("dy_m", 0.0))
            self._pending_dyaw_rad += float(data.get("dyaw_rad", 0.0))
            self._latest_speed_mps = float(data.get("speed_mps", 0.0))
            self._latest_steering_deg = float(data.get("steering_deg", 0.0))
            self._latest_accelerator_pressed = bool(data.get("accelerator_pressed", False))
            self._latest_accelerator_pedal = float(data.get("accelerator_pedal", 0.0))
            self._latest_accelerator_pedal_raw = int(data.get("accelerator_pedal_raw", 0))
            self._latest_brake_pressed = bool(data.get("brake_pressed", False) or data.get("brake_lights", False))
            self._latest_brake_lights = bool(data.get("brake_lights", self._latest_brake_pressed))
            self._has_sample = bool(data.get("valid", False))

    def _integrate_can_sample(self, can_state: _CanEgoState, dt: float, now: float) -> None:
        wheel_avg_kph = sum(can_state.wheel_speeds_kph) / 4.0
        direction = -1.0 if can_state.moving_backward else 1.0
        speed_mps = direction * wheel_avg_kph * KPH_TO_MS

        if self.angle_source == "mdps":
            steering_deg = can_state.mdps_steering_deg
        else:
            steering_deg = can_state.steering_sensor_deg
        if self.invert_steer:
            steering_deg *= -1.0

        road_angle_rad = math.radians(steering_deg) / max(self.steer_ratio, 1e-6)
        yaw_rate = speed_mps / max(self.wheelbase_m, 1e-6) * math.tan(road_angle_rad)
        dyaw = yaw_rate * dt
        dx = speed_mps * math.cos(0.5 * dyaw) * dt
        dy = speed_mps * math.sin(0.5 * dyaw) * dt

        stopped = abs(speed_mps) < self.stop_speed_threshold_mps
        reset = False
        if stopped:
            if self._stopped_since is None:
                self._stopped_since = now
            if not self._stopped_reset_sent and now - self._stopped_since >= self.stop_reset_sec:
                reset = True
                self._stopped_reset_sent = True
        else:
            self._stopped_since = None
            self._stopped_reset_sent = False

        with self._lock:
            if reset:
                self._pending_dx_m = 0.0
                self._pending_dy_m = 0.0
                self._pending_dyaw_rad = 0.0
                self._reset_pending = True
            else:
                self._pending_dx_m += dx
                self._pending_dy_m += dy
            self._pending_dyaw_rad += dyaw
            self._latest_speed_mps = speed_mps
            self._latest_steering_deg = steering_deg
            self._latest_accelerator_pressed = can_state.accelerator_pressed
            self._latest_accelerator_pedal = can_state.accelerator_pedal
            self._latest_accelerator_pedal_raw = can_state.accelerator_pedal_raw
            self._latest_brake_pressed = can_state.brake_pressed
            self._latest_brake_lights = can_state.brake_lights
            self._has_sample = can_state.seen_wheel_speeds or can_state.seen_steering

    def _update_can_state(self, can_state: _CanEgoState, can_msgs) -> None:
        for addr, dat, bus in can_msgs:
            if bus != self.bus:
                continue
            if addr == ADDR_WHEEL_SPEEDS and len(dat) >= 16:
                can_state.wheel_speeds_kph = [
                    self._u16_le(dat, 8) * 0.03125,
                    self._u16_le(dat, 10) * 0.03125,
                    self._u16_le(dat, 12) * 0.03125,
                    self._u16_le(dat, 14) * 0.03125,
                ]
                # Direction bits are in the motion-status byte for this DBC.
                can_state.moving_backward = bool(dat[7] & 0x0A)
                can_state.seen_wheel_speeds = True
            elif addr == ADDR_STEERING_SENSORS and len(dat) >= 6:
                # DBC scale is -0.1, and openpilot's EV6 tools multiply by -1.
                can_state.steering_sensor_deg = self._s16_le(dat, 3) * 0.1
                can_state.seen_steering = True
            elif addr == ADDR_MDPS and len(dat) >= 18:
                can_state.mdps_steering_deg = self._s16_le(dat, 16) * 0.1
                can_state.seen_steering = True
            elif addr == ADDR_ACCELERATOR and len(dat) >= 6:
                raw = int(dat[5])
                can_state.accelerator_pedal_raw = raw
                can_state.accelerator_pedal = raw / 255.0
                can_state.accelerator_pressed = raw > 0
            elif addr == ADDR_ACCELERATOR_ALT and len(dat) >= 15:
                raw = ((int(dat[12]) >> 7) | (int(dat[13]) << 1) | ((int(dat[14]) & 0x01) << 9)) & 0x3FF
                can_state.accelerator_pedal_raw = raw
                can_state.accelerator_pedal = min(raw / 1022.0, 1.0)
                can_state.accelerator_pressed = raw > 0 or self._bit_is_set(dat, 103) or self._bit_is_set(dat, 112)
            elif addr == ADDR_ACCELERATOR_BRAKE_ALT and len(dat) >= 23:
                can_state.accelerator_pressed = self._bit_is_set(dat, 176)
                if can_state.accelerator_pressed:
                    can_state.accelerator_pedal = max(can_state.accelerator_pedal, 1.0)
                can_state.brake_pressed = can_state.brake_pressed or self._bit_is_set(dat, 32)
                can_state.brake_lights = can_state.brake_lights or can_state.brake_pressed
            elif addr == ADDR_TCS and len(dat) >= 11:
                brake_light = (dat[9] >> 2) & 0x3
                driver_braking = (dat[10] >> 6) & 0x1
                driver_braking_low_sens = (dat[10] >> 4) & 0x1
                can_state.brake_pressed = bool(driver_braking or driver_braking_low_sens or brake_light)
                can_state.brake_lights = bool(brake_light or can_state.brake_pressed)

    @staticmethod
    def _u16_le(dat: bytes, offset: int) -> int:
        return dat[offset] | (dat[offset + 1] << 8)

    @staticmethod
    def _bit_is_set(dat: bytes, bit: int) -> bool:
        byte_index = bit // 8
        bit_index = bit % 8
        return byte_index < len(dat) and bool(dat[byte_index] & (1 << bit_index))

    @classmethod
    def _s16_le(cls, dat: bytes, offset: int) -> int:
        val = cls._u16_le(dat, offset)
        return val - 0x10000 if val & 0x8000 else val
