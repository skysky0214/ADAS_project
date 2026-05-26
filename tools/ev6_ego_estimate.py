#!/usr/bin/env python3
"""
EV6 ego odometry estimator from Panda CAN-FD + opendbc.

This estimates 2D pose using wheel speeds and steering angle:
  v = average wheel speed
  road_wheel_angle = steering_wheel_angle / steer_ratio
  yaw_rate = v / wheelbase * tan(road_wheel_angle)

Pose origin is the rear axle center at program start. Vehicle center is reported
using the provided body dimensions.
"""

import argparse
import csv
import math
import os
import signal
import sys
import time


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENPILOT_ROOT = os.path.join(ROOT, "openpilot")
sys.path.insert(0, OPENPILOT_ROOT)
sys.path.insert(0, os.path.join(OPENPILOT_ROOT, "opendbc_repo"))

from opendbc.can import CANParser  # noqa: E402
from panda import Panda  # noqa: E402


DBC_NAME = "hyundai_canfd_generated"
KPH_TO_MS = 1000.0 / 3600.0


def make_parser(bus: int) -> CANParser:
  msgs = [
    ("WHEEL_SPEEDS", 0),
    ("STEERING_SENSORS", 0),
    ("MDPS", 0),
  ]
  return CANParser(DBC_NAME, msgs, bus)


def configure_panda(panda: Panda, bus: int, can_speed: int, data_speed: int):
  panda.set_safety_mode(0, 0)  # SAFETY_SILENT, receive-only
  panda.set_can_speed_kbps(bus, can_speed)
  panda.set_can_data_speed_kbps(bus, data_speed)
  panda.set_canfd_auto(bus, True)


def wrap_pi(angle: float) -> float:
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def vehicle_center_from_rear_axle(x: float, y: float, yaw: float, wheelbase: float,
                                  front_overhang: float, rear_overhang: float):
  # Rear axle is rear_overhang meters ahead of rear bumper. The body center is
  # halfway along overall body length from rear bumper.
  rear_axle_to_center = ((front_overhang + wheelbase + rear_overhang) * 0.5) - rear_overhang
  return (
    x + rear_axle_to_center * math.cos(yaw),
    y + rear_axle_to_center * math.sin(yaw),
  )


def parse_args():
  parser = argparse.ArgumentParser(description="Estimate EV6 ego odometry from Panda CAN-FD")
  parser.add_argument("--bus", type=int, default=0, help="Panda bus with ECAN/WHEEL_SPEEDS/MDPS")
  parser.add_argument("--can-speed", type=int, default=500, help="Nominal CAN speed in kbps")
  parser.add_argument("--data-speed", type=int, default=2000, help="CAN-FD data speed in kbps")
  parser.add_argument("--no-config", action="store_true", help="Do not configure Panda CAN-FD speeds")
  parser.add_argument("--print-hz", type=float, default=20.0, help="Console print rate")

  parser.add_argument("--length", type=float, default=4.695, help="Overall vehicle length in meters")
  parser.add_argument("--width", type=float, default=1.880, help="Overall vehicle width in meters")
  parser.add_argument("--height", type=float, default=1.550, help="Overall vehicle height in meters")
  parser.add_argument("--wheelbase", type=float, default=2.900, help="Wheelbase in meters")
  parser.add_argument("--front-overhang", type=float, default=0.870, help="Front overhang in meters")
  parser.add_argument("--rear-overhang", type=float, default=0.785, help="Rear overhang in meters")
  parser.add_argument("--steer-ratio", type=float, default=16.0, help="Steering wheel angle / road wheel angle")
  parser.add_argument("--angle-source", choices=("sensor", "mdps"), default="sensor",
                      help="Use STEERING_SENSORS or MDPS angle")
  parser.add_argument("--invert-steer", action="store_true", help="Invert decoded steering sign")
  parser.add_argument("--max-dt", type=float, default=0.1, help="Clamp integration dt to this many seconds")
  parser.add_argument("--log-csv", help="Optional CSV output path")
  return parser.parse_args()


def main():
  args = parse_args()
  stop = False

  def handle_sigint(_sig, _frame):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, handle_sigint)

  parser = make_parser(args.bus)
  panda = Panda()
  print("Panda connected:", panda.get_serial())
  if not args.no_config:
    configure_panda(panda, args.bus, args.can_speed, args.data_speed)
    print(f"Configured CAN-FD: nominal={args.can_speed} kbps data={args.data_speed} kbps bus={args.bus}")

  # Report geometry. The user-provided overhangs and total length can differ
  # slightly depending on trim/source, so odometry uses wheelbase for dynamics.
  derived_length = args.front_overhang + args.wheelbase + args.rear_overhang
  print(
    f"Geometry: L={args.length:.3f}m W={args.width:.3f}m H={args.height:.3f}m "
    f"WB={args.wheelbase:.3f}m FOH={args.front_overhang:.3f}m ROH={args.rear_overhang:.3f}m "
    f"derived_L={derived_length:.3f}m steer_ratio={args.steer_ratio:.2f}"
  )

  csv_file = None
  csv_writer = None
  if args.log_csv:
    csv_file = open(args.log_csv, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=[
      "t", "x_rear_m", "y_rear_m", "x_center_m", "y_center_m", "yaw_deg",
      "v_mps", "wheel_avg_kph", "steer_deg", "road_angle_deg", "yaw_rate_dps",
      "distance_m", "w1_kph", "w2_kph", "w3_kph", "w4_kph",
    ])
    csv_writer.writeheader()

  x = 0.0
  y = 0.0
  yaw = 0.0
  distance = 0.0
  last_t = time.monotonic()
  start_t = last_t
  next_print = 0.0
  print_dt = 1.0 / args.print_hz

  try:
    while not stop:
      can_msgs = panda.can_recv()
      if can_msgs:
        parser.update([int(time.monotonic() * 1e9), can_msgs])

      now = time.monotonic()
      dt = min(max(0.0, now - last_t), args.max_dt)
      last_t = now

      ws = parser.vl["WHEEL_SPEEDS"]
      ss = parser.vl["STEERING_SENSORS"]
      mdps = parser.vl["MDPS"]

      wheel_speeds_kph = [
        ws["WHEEL_SPEED_1"],
        ws["WHEEL_SPEED_2"],
        ws["WHEEL_SPEED_3"],
        ws["WHEEL_SPEED_4"],
      ]
      wheel_avg_kph = sum(wheel_speeds_kph) / 4.0
      direction = -1.0 if ws["MOVING_BACKWARD"] or ws["MOVING_BACKWARD2"] else 1.0
      v = direction * wheel_avg_kph * KPH_TO_MS

      if args.angle_source == "mdps":
        steer_deg = -mdps["STEERING_ANGLE_2"]
      else:
        steer_deg = -ss["STEERING_ANGLE"]
      if args.invert_steer:
        steer_deg *= -1.0

      road_angle_rad = math.radians(steer_deg / args.steer_ratio)
      yaw_rate = v / args.wheelbase * math.tan(road_angle_rad)

      # Midpoint integration is stable enough for low-rate CAN odometry.
      yaw_mid = yaw + 0.5 * yaw_rate * dt
      x += v * math.cos(yaw_mid) * dt
      y += v * math.sin(yaw_mid) * dt
      yaw = wrap_pi(yaw + yaw_rate * dt)
      distance += abs(v) * dt

      center_x, center_y = vehicle_center_from_rear_axle(
        x, y, yaw, args.wheelbase, args.front_overhang, args.rear_overhang
      )

      if csv_writer is not None:
        csv_writer.writerow({
          "t": now - start_t,
          "x_rear_m": x,
          "y_rear_m": y,
          "x_center_m": center_x,
          "y_center_m": center_y,
          "yaw_deg": math.degrees(yaw),
          "v_mps": v,
          "wheel_avg_kph": wheel_avg_kph,
          "steer_deg": steer_deg,
          "road_angle_deg": math.degrees(road_angle_rad),
          "yaw_rate_dps": math.degrees(yaw_rate),
          "distance_m": distance,
          "w1_kph": wheel_speeds_kph[0],
          "w2_kph": wheel_speeds_kph[1],
          "w3_kph": wheel_speeds_kph[2],
          "w4_kph": wheel_speeds_kph[3],
        })

      if now >= next_print:
        next_print = now + print_dt
        print(
          f"t={now - start_t:7.2f}s "
          f"rear=({x:8.3f},{y:8.3f})m center=({center_x:8.3f},{center_y:8.3f})m "
          f"yaw={math.degrees(yaw):7.2f}deg "
          f"v={v:6.2f}m/s wheel={wheel_avg_kph:6.2f}kph "
          f"steer={steer_deg:7.2f}deg road={math.degrees(road_angle_rad):6.2f}deg "
          f"yaw_rate={math.degrees(yaw_rate):7.2f}deg/s dist={distance:8.2f}m"
        )

      time.sleep(0.001)
  finally:
    if csv_file is not None:
      csv_file.close()
    panda.set_safety_mode(0, 0)
    panda.close()
    print("Stopped.")


if __name__ == "__main__":
  main()
