#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import signal
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
    return byte_index < len(dat) and bool(dat[byte_index] & (1 << (bit % 8)))


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


class VehicleTopicsPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("vehicle_topics_publisher")
        self.args = args
        self.state = CanEgoState()
        self.panda = Panda()
        self.panda.set_safety_mode(SAFETY_SILENT, 0)
        if not args.no_config:
            self.panda.set_can_speed_kbps(args.bus, args.can_speed)
            self.panda.set_can_data_speed_kbps(args.bus, args.data_speed)
            self.panda.set_canfd_auto(args.bus, True)

        self.ego_pub = self.create_publisher(String, args.ego_topic, 100)
        self.raw_pub = self.create_publisher(String, args.raw_topic, 100)
        self.last_t = time.monotonic()
        self.last_ego_emit = 0.0
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dyaw = 0.0
        self.timer = self.create_timer(0.001, self.step)
        self.get_logger().info(
            f"Publishing {args.ego_topic} and {args.raw_topic} from Panda bus {args.bus}"
        )

    def step(self) -> None:
        now = time.monotonic()
        can_msgs = self.panda.can_recv()
        if can_msgs:
            update_can_state(self.state, can_msgs, self.args.bus)
            stamp = time.time()
            for addr, dat, bus in can_msgs:
                if bus != self.args.bus:
                    continue
                self.raw_pub.publish(
                    String(
                        data=json.dumps(
                            {"ts": stamp, "address": addr, "bytes": dat.hex(), "bus": bus},
                            separators=(",", ":"),
                        )
                    )
                )

        dt = min(max(now - self.last_t, 0.0), self.args.max_dt)
        self.last_t = now
        delta, reset = self.integrate(dt, now)
        if reset:
            self.pending_dx = 0.0
            self.pending_dy = 0.0
            self.pending_dyaw = 0.0
        else:
            self.pending_dx += delta["dx_m"]
            self.pending_dy += delta["dy_m"]
            self.pending_dyaw += delta["dyaw_rad"]

        if reset or now - self.last_ego_emit >= 1.0 / max(self.args.emit_hz, 1e-6):
            payload = {
                **delta,
                "dx_m": self.pending_dx,
                "dy_m": self.pending_dy,
                "dyaw_rad": self.pending_dyaw,
                "reset": reset,
            }
            self.ego_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
            self.pending_dx = 0.0
            self.pending_dy = 0.0
            self.pending_dyaw = 0.0
            self.last_ego_emit = now

    def integrate(self, dt: float, now: float) -> tuple[dict, bool]:
        state = self.state
        wheel_avg_kph = sum(state.wheel_speeds_kph) / 4.0
        speed_mps = (-1.0 if state.moving_backward else 1.0) * wheel_avg_kph * KPH_TO_MS
        steering_deg = state.mdps_steering_deg if self.args.angle_source == "mdps" else state.steering_sensor_deg
        if self.args.invert_steer:
            steering_deg *= -1.0
        road_angle_rad = math.radians(steering_deg) / max(self.args.steer_ratio, 1e-6)
        yaw_rate = speed_mps / max(self.args.wheelbase, 1e-6) * math.tan(road_angle_rad)
        dyaw = yaw_rate * dt

        stopped = abs(speed_mps) < self.args.stop_speed_threshold
        reset = False
        if stopped:
            if state.stopped_since is None:
                state.stopped_since = now
            if not state.stopped_reset_sent and now - state.stopped_since >= self.args.stop_reset_sec:
                reset = True
                state.stopped_reset_sent = True
        else:
            state.stopped_since = None
            state.stopped_reset_sent = False

        return (
            {
                "dx_m": speed_mps * math.cos(0.5 * dyaw) * dt,
                "dy_m": speed_mps * math.sin(0.5 * dyaw) * dt,
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

    def destroy_node(self) -> bool:
        try:
            self.panda.set_safety_mode(SAFETY_SILENT, 0)
            self.panda.close()
        finally:
            return super().destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish EV6 Panda CAN ego data to ROS topics for bag recording")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--can-speed", type=int, default=500)
    parser.add_argument("--data-speed", type=int, default=2000)
    parser.add_argument("--no-config", action="store_true")
    parser.add_argument("--wheelbase", type=float, default=2.900)
    parser.add_argument("--steer-ratio", type=float, default=DEFAULT_STEER_RATIO)
    parser.add_argument("--angle-source", choices=("sensor", "mdps"), default="sensor")
    parser.add_argument("--invert-steer", action="store_true")
    parser.add_argument("--max-dt", type=float, default=0.1)
    parser.add_argument("--stop-speed-threshold", type=float, default=0.05)
    parser.add_argument("--stop-reset-sec", type=float, default=0.7)
    parser.add_argument("--emit-hz", type=float, default=100.0)
    parser.add_argument("--ego-topic", default="/vehicle/ego_motion")
    parser.add_argument("--raw-topic", default="/vehicle/can/raw")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = VehicleTopicsPublisher(args)

    stop = False

    def handle_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
