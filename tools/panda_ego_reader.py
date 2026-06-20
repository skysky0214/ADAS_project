#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import dataclass, field

from panda import Panda


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


@dataclass
class CanEgoState:
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
    stopped_since: float | None = None
    stopped_reset_sent: bool = False


def u16_le(dat: bytes, offset: int) -> int:
    return dat[offset] | (dat[offset + 1] << 8)


def s16_le(dat: bytes, offset: int) -> int:
    val = u16_le(dat, offset)
    return val - 0x10000 if val & 0x8000 else val


def bit_is_set(dat: bytes, bit: int) -> bool:
    byte_index = bit // 8
    bit_index = bit % 8
    return byte_index < len(dat) and bool(dat[byte_index] & (1 << bit_index))


def update_can_state(state: CanEgoState, can_msgs, bus: int) -> None:
    for addr, dat, msg_bus in can_msgs:
        if msg_bus != bus:
            continue
        if addr == ADDR_WHEEL_SPEEDS and len(dat) >= 16:
            state.wheel_speeds_kph = [
                u16_le(dat, 8) * 0.03125,
                u16_le(dat, 10) * 0.03125,
                u16_le(dat, 12) * 0.03125,
                u16_le(dat, 14) * 0.03125,
            ]
            state.moving_backward = bool(dat[7] & 0x0A)
            state.seen_wheel_speeds = True
        elif addr == ADDR_STEERING_SENSORS and len(dat) >= 6:
            # DBC scale is -0.1; openpilot's EV6 utility uses -STEERING_ANGLE.
            state.steering_sensor_deg = s16_le(dat, 3) * 0.1
            state.seen_steering = True
        elif addr == ADDR_MDPS and len(dat) >= 18:
            state.mdps_steering_deg = s16_le(dat, 16) * 0.1
            state.seen_steering = True
        elif addr == ADDR_ACCELERATOR and len(dat) >= 6:
            raw = int(dat[5])
            state.accelerator_pedal_raw = raw
            state.accelerator_pedal = raw / 255.0
            state.accelerator_pressed = raw > 0
        elif addr == ADDR_ACCELERATOR_ALT and len(dat) >= 15:
            raw = ((int(dat[12]) >> 7) | (int(dat[13]) << 1) | ((int(dat[14]) & 0x01) << 9)) & 0x3FF
            state.accelerator_pedal_raw = raw
            state.accelerator_pedal = min(raw / 1022.0, 1.0)
            state.accelerator_pressed = raw > 0 or bit_is_set(dat, 103) or bit_is_set(dat, 112)
        elif addr == ADDR_ACCELERATOR_BRAKE_ALT and len(dat) >= 23:
            state.accelerator_pressed = bit_is_set(dat, 176)
            if state.accelerator_pressed:
                state.accelerator_pedal = max(state.accelerator_pedal, 1.0)
            state.brake_pressed = state.brake_pressed or bit_is_set(dat, 32)
            state.brake_lights = state.brake_lights or state.brake_pressed
        elif addr == ADDR_TCS and len(dat) >= 11:
            brake_light = (dat[9] >> 2) & 0x3
            driver_braking = (dat[10] >> 6) & 0x1
            driver_braking_low_sens = (dat[10] >> 4) & 0x1
            state.brake_pressed = bool(driver_braking or driver_braking_low_sens or brake_light)
            state.brake_lights = bool(brake_light or state.brake_pressed)


def integrate(state: CanEgoState, args: argparse.Namespace, dt: float, now: float) -> tuple[dict, bool]:
    wheel_avg_kph = sum(state.wheel_speeds_kph) / 4.0
    direction = -1.0 if state.moving_backward else 1.0
    speed_mps = direction * wheel_avg_kph * KPH_TO_MS

    steering_deg = state.mdps_steering_deg if args.angle_source == "mdps" else state.steering_sensor_deg
    if args.invert_steer:
        steering_deg *= -1.0

    road_angle_rad = math.radians(steering_deg) / max(args.steer_ratio, 1e-6)
    yaw_rate = speed_mps / max(args.wheelbase, 1e-6) * math.tan(road_angle_rad)
    dyaw = yaw_rate * dt
    dx = speed_mps * math.cos(0.5 * dyaw) * dt
    dy = speed_mps * math.sin(0.5 * dyaw) * dt

    stopped = abs(speed_mps) < args.stop_speed_threshold
    reset = False
    if stopped:
        if state.stopped_since is None:
            state.stopped_since = now
        if not state.stopped_reset_sent and now - state.stopped_since >= args.stop_reset_sec:
            reset = True
            state.stopped_reset_sent = True
    else:
        state.stopped_since = None
        state.stopped_reset_sent = False

    return (
        {
            "dx_m": dx,
            "dy_m": dy,
            "dyaw_rad": dyaw,
            "speed_mps": speed_mps,
            "steering_deg": steering_deg,
            "accelerator_pressed": state.accelerator_pressed,
            "accelerator_pedal": state.accelerator_pedal,
            "accelerator_pedal_raw": state.accelerator_pedal_raw,
            "brake_pressed": state.brake_pressed,
            "brake_lights": state.brake_lights,
            "valid": state.seen_wheel_speeds or state.seen_steering,
        },
        reset,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stream EV6 ego motion from Panda CAN as JSON lines")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--can-speed", type=int, default=500)
    parser.add_argument("--data-speed", type=int, default=2000)
    parser.add_argument("--no-config", action="store_true")
    parser.add_argument("--wheelbase", type=float, default=2.900)
    parser.add_argument("--steer-ratio", type=float, default=DEFAULT_STEER_RATIO, help="Steering wheel angle / road wheel angle")
    parser.add_argument("--angle-source", choices=("sensor", "mdps"), default="sensor")
    parser.add_argument("--invert-steer", action="store_true")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--stop-speed-threshold", type=float, default=0.05)
    parser.add_argument("--stop-reset-sec", type=float, default=0.7)
    parser.add_argument("--emit-hz", type=float, default=100.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    stop = False

    def handle_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    panda = None
    try:
        state = CanEgoState()
        panda = Panda()
        panda.set_safety_mode(SAFETY_SILENT, 0)
        if not args.no_config:
            panda.set_can_speed_kbps(args.bus, args.can_speed)
            panda.set_can_data_speed_kbps(args.bus, args.data_speed)
            panda.set_canfd_auto(args.bus, True)

        print("EGO_READY", flush=True)

        last_t = time.monotonic()
        last_emit = 0.0
        emit_dt = 1.0 / max(args.emit_hz, 1e-6)
        pending_dx = 0.0
        pending_dy = 0.0
        pending_dyaw = 0.0
        latest_speed = 0.0
        latest_steering = 0.0
        latest_accelerator_pressed = False
        latest_accelerator_pedal = 0.0
        latest_accelerator_pedal_raw = 0
        latest_valid = False

        while not stop:
            can_msgs = panda.can_recv()
            now = time.monotonic()
            if can_msgs:
                update_can_state(state, can_msgs, args.bus)

            dt = min(max(now - last_t, 0.0), args.max_dt)
            last_t = now
            delta, reset = integrate(state, args, dt, now)
            if reset:
                pending_dx = 0.0
                pending_dy = 0.0
                pending_dyaw = 0.0
            else:
                pending_dx += delta["dx_m"]
                pending_dy += delta["dy_m"]
                pending_dyaw += delta["dyaw_rad"]
            latest_speed = delta["speed_mps"]
            latest_steering = delta["steering_deg"]
            latest_accelerator_pressed = delta["accelerator_pressed"]
            latest_accelerator_pedal = delta["accelerator_pedal"]
            latest_accelerator_pedal_raw = delta["accelerator_pedal_raw"]
            latest_valid = delta["valid"]

            if reset or now - last_emit >= emit_dt:
                print(
                    "EGO "
                    + json.dumps(
                        {
                            "dx_m": pending_dx,
                            "dy_m": pending_dy,
                            "dyaw_rad": pending_dyaw,
                            "speed_mps": latest_speed,
                            "steering_deg": latest_steering,
                            "accelerator_pressed": latest_accelerator_pressed,
                            "accelerator_pedal": latest_accelerator_pedal,
                            "accelerator_pedal_raw": latest_accelerator_pedal_raw,
                            "brake_pressed": state.brake_pressed,
                            "brake_lights": state.brake_lights,
                            "valid": latest_valid,
                            "reset": reset,
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                pending_dx = 0.0
                pending_dy = 0.0
                pending_dyaw = 0.0
                last_emit = now

            time.sleep(0.001)
    except Exception as exc:
        print(f"EGO_ERROR {type(exc).__name__}: {exc}", flush=True)
        return 1
    finally:
        if panda is not None:
            try:
                panda.set_safety_mode(SAFETY_SILENT, 0)
                panda.close()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
