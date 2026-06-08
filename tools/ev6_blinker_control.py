#!/usr/bin/env python3
"""
EV6 CAN-FD blinker command test using SPAS1/SPAS2.

This is TX-only by design. It continuously sends SPAS1/SPAS2 while active and
sends an "off" command before exit.
"""

from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import time


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENPILOT_ROOT = os.path.join(ROOT, "openpilot")
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, OPENPILOT_ROOT)
sys.path.insert(0, os.path.join(OPENPILOT_ROOT, "opendbc_repo"))

from opendbc.can.packer import CANPacker  # noqa: E402
from panda import Panda  # noqa: E402
from ev6_sensor_module import (  # noqa: E402
  BODY_DBC,
  SAFETY_NOOUTPUT,
  SAFETY_SILENT,
  effective_blinker_command,
  send_spas_blinker,
)


COMMANDS = ("off", "left", "right", "hazard")


def parse_args():
  parser = argparse.ArgumentParser(description="EV6 SPAS blinker TX test")
  parser.add_argument("command", choices=COMMANDS + ("interactive",),
                      help="Blinker command to send, or interactive keyboard mode")
  parser.add_argument("--bus", type=int, default=0, help="TX bus for SPAS1/SPAS2")
  parser.add_argument("--hz", type=float, default=20.0, help="TX rate")
  parser.add_argument("--hazard-hz", type=float, default=0.75, help="Hazard blink cycle rate")
  parser.add_argument("--duration", type=float, default=5.0,
                      help="Send duration in seconds. Use 0 for until Ctrl-C.")
  parser.add_argument("--can-speed", type=int, default=500)
  parser.add_argument("--data-speed", type=int, default=2000)
  parser.add_argument("--disable-fd", action="store_true", help="Send classic CAN instead of CAN-FD")
  parser.add_argument("--no-config", action="store_true", help="Do not configure Panda speeds")
  parser.add_argument("--safety", choices=("nooutput", "unchanged"), default="nooutput",
                      help="nooutput keeps the relay closed. alloutput is intentionally not supported here.")
  return parser.parse_args()


def configure_bus_only(panda: Panda, bus: int, can_speed: int, data_speed: int):
  panda.set_can_speed_kbps(bus, can_speed)
  panda.set_can_data_speed_kbps(bus, data_speed)
  panda.set_canfd_auto(bus, True)


def parse_command(raw: str, current: str) -> str:
  raw = raw.strip().lower()
  if raw in ("l", "left"):
    return "left"
  if raw in ("r", "right"):
    return "right"
  if raw in ("h", "hazard"):
    return "hazard"
  if raw in ("0", "o", "off"):
    return "off"
  if raw in ("q", "quit", "exit"):
    raise KeyboardInterrupt
  return current


def poll_interactive_command(current: str) -> str:
  readable, _, _ = select.select([sys.stdin], [], [], 0.0)
  if not readable:
    return current
  return parse_command(sys.stdin.readline(), current)


def main():
  args = parse_args()
  stop = False

  def handle_sigint(_sig, _frame):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, handle_sigint)

  panda = Panda()
  print("Panda connected:", panda.get_serial())
  if not args.no_config:
    configure_bus_only(panda, args.bus, args.can_speed, args.data_speed)
  safety_changed = args.safety != "unchanged"
  if args.safety == "nooutput":
    panda.set_safety_mode(SAFETY_NOOUTPUT, args.bus)
    print(f"Panda safety: noOutput, relay remains closed, SPAS TX bus={args.bus}")
  else:
    print("Panda safety unchanged; relay/safety mode will not be touched")

  packer = CANPacker(BODY_DBC)
  fd = not args.disable_fd
  command = "off" if args.command == "interactive" else args.command
  next_tx = 0.0
  start_t = time.monotonic()
  tx_dt = 1.0 / args.hz

  try:
    if args.command == "interactive":
      print("interactive commands: l=left r=right h=hazard 0/off=off q=quit then Enter")
    print(f"Sending {command} on bus {args.bus} at {args.hz:.1f} Hz fd={fd}")
    while not stop:
      now = time.monotonic()
      if args.duration > 0 and args.command != "interactive" and now - start_t >= args.duration:
        break
      if now >= next_tx:
        next_tx = now + tx_dt
        send_spas_blinker(
          panda,
          packer,
          effective_blinker_command(command, now, args.hazard_hz),
          args.bus,
          fd=fd,
        )
      if args.command == "interactive":
        new_command = poll_interactive_command(command)
        if new_command != command:
          command = new_command
          print(f"Sending {command}")
      time.sleep(0.001)
  finally:
    try:
      for _ in range(5):
        send_spas_blinker(panda, packer, "off", args.bus, fd=fd)
        time.sleep(0.02)
    finally:
      if safety_changed:
        panda.set_safety_mode(SAFETY_SILENT, 0)
      panda.close()
      print("Stopped.")


if __name__ == "__main__":
  main()
