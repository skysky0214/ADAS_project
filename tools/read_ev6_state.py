#!/usr/bin/env python3
"""
Read EV6 CAN-FD steering angle and wheel speeds through Panda.

Defaults match the current Red Panda wiring used in this workspace:
  CAN2 / ECAN -> panda bus 0
  CAN1 / ACAN -> panda bus 1
"""

import argparse
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


def make_parser(bus: int) -> CANParser:
  # Frequency 0 disables rate validity requirements; this is a live diagnostic tool.
  msgs = [
    ("WHEEL_SPEEDS", 0),
    ("STEERING_SENSORS", 0),
    ("MDPS", 0),
  ]
  return CANParser(DBC_NAME, msgs, bus)


def parse_args():
  parser = argparse.ArgumentParser(description="Read EV6 steering angle and wheel speeds from Panda")
  parser.add_argument("--buses", default="0", help="Comma-separated panda buses to decode, e.g. 0 or 0,1")
  parser.add_argument("--can-speed", type=int, default=500, help="Nominal CAN speed in kbps")
  parser.add_argument("--data-speed", type=int, default=2000, help="CAN-FD data speed in kbps")
  parser.add_argument("--print-hz", type=float, default=10.0, help="Print rate")
  parser.add_argument("--no-config", action="store_true", help="Do not configure Panda CAN-FD speeds")
  return parser.parse_args()


def configure_panda(panda: Panda, buses: list[int], can_speed: int, data_speed: int):
  panda.set_safety_mode(0, 0)  # SAFETY_SILENT, receive-only
  for bus in buses:
    panda.set_can_speed_kbps(bus, can_speed)
    panda.set_can_data_speed_kbps(bus, data_speed)
    panda.set_canfd_auto(bus, True)


def msg_seen(parser: CANParser, msg_name: str) -> bool:
  return parser.vl[msg_name]["COUNTER"] != 0 or msg_name in parser.vl_all


def fmt_float(v: float, width: int = 7, precision: int = 2) -> str:
  return f"{v:{width}.{precision}f}"


def main():
  args = parse_args()
  buses = [int(x.strip()) for x in args.buses.split(",") if x.strip()]
  parsers = {bus: make_parser(bus) for bus in buses}
  stop = False

  def handle_sigint(_sig, _frame):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, handle_sigint)

  panda = Panda()
  print("Panda connected:", panda.get_serial())
  if not args.no_config:
    configure_panda(panda, buses, args.can_speed, args.data_speed)
    print(f"Configured CAN-FD: nominal={args.can_speed} kbps data={args.data_speed} kbps buses={buses}")

  next_print = 0.0
  print_dt = 1.0 / args.print_hz

  try:
    while not stop:
      can_msgs = panda.can_recv()
      if can_msgs:
        now_nanos = int(time.monotonic() * 1e9)
        update = [now_nanos, can_msgs]
        for parser in parsers.values():
          parser.update(update)

      now = time.monotonic()
      if now >= next_print:
        next_print = now + print_dt
        for bus, parser in parsers.items():
          ws = parser.vl["WHEEL_SPEEDS"]
          ss = parser.vl["STEERING_SENSORS"]
          mdps = parser.vl["MDPS"]

          wheel_avg = (
            ws["WHEEL_SPEED_1"] +
            ws["WHEEL_SPEED_2"] +
            ws["WHEEL_SPEED_3"] +
            ws["WHEEL_SPEED_4"]
          ) / 4.0

          # openpilot's CAN-FD CarState multiplies these angles by -1.
          steering_sensor_angle = -ss["STEERING_ANGLE"]
          mdps_angle = -mdps["STEERING_ANGLE_2"]

          print(
            f"bus={bus} "
            f"steer_sensor={fmt_float(steering_sensor_angle)} deg "
            f"rate={fmt_float(ss['STEERING_RATE'])} deg/s "
            f"mdps_angle={fmt_float(mdps_angle)} deg "
            f"torque_col={fmt_float(mdps['STEERING_COL_TORQUE'], precision=1)} "
            f"torque_eps={fmt_float(mdps['STEERING_OUT_TORQUE'], precision=1)} "
            f"wheel_kph=["
            f"{fmt_float(ws['WHEEL_SPEED_1'])},"
            f"{fmt_float(ws['WHEEL_SPEED_2'])},"
            f"{fmt_float(ws['WHEEL_SPEED_3'])},"
            f"{fmt_float(ws['WHEEL_SPEED_4'])}"
            f"] avg={fmt_float(wheel_avg)}"
          )

      time.sleep(0.001)
  finally:
    panda.set_safety_mode(0, 0)
    panda.close()
    print("Stopped.")


if __name__ == "__main__":
  main()
