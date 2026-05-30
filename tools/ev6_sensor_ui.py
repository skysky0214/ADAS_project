#!/usr/bin/env python3
"""
Live EV6 ADAS sensor UI from Panda CAN-FD + opendbc.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from collections import deque

import pygame


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
  EV6SensorReader,
  configure_panda,
  send_spas_blinker,
)


COL_BG = (14, 16, 19)
COL_PANEL = (24, 27, 31)
COL_LINE = (66, 72, 80)
COL_TEXT = (229, 234, 240)
COL_MUTED = (148, 158, 170)
COL_GREEN = (78, 220, 132)
COL_YELLOW = (255, 205, 75)
COL_RED = (255, 82, 82)
COL_BLUE = (76, 172, 255)
COL_CAR = (228, 232, 236)


def clamp(v, lo, hi):
  return max(lo, min(hi, v))


def distance_color(d: float, active: bool):
  if not active:
    return (52, 57, 64)
  if d <= 1.5:
    return COL_RED
  if d <= 5.0:
    return COL_YELLOW
  return COL_GREEN


def draw_text(screen, font, x, y, text, color=COL_TEXT):
  surf = font.render(text, True, color)
  screen.blit(surf, (x, y))


def draw_panel(screen, rect):
  pygame.draw.rect(screen, COL_PANEL, rect, border_radius=6)
  pygame.draw.rect(screen, COL_LINE, rect, 1, border_radius=6)


def draw_indicator(screen, center, label, active, color):
  x, y = center
  c = color if active else (55, 60, 67)
  pygame.draw.circle(screen, c, (x, y), 16)
  pygame.draw.circle(screen, (96, 104, 114), (x, y), 16, 1)


def draw_arrow(screen, points, active, color):
  c = color if active else (50, 55, 62)
  pygame.draw.polygon(screen, c, points)
  pygame.draw.polygon(screen, (96, 104, 114), points, 1)


def draw_car(screen, rect, state, control_cmd):
  draw_panel(screen, rect)
  x, y, w, h = rect
  cx = x + w // 2
  cy = y + h // 2 + 15

  car_w = 118
  car_h = 290
  car = pygame.Rect(cx - car_w // 2, cy - car_h // 2, car_w, car_h)
  pygame.draw.rect(screen, (36, 40, 46), car, border_radius=12)
  pygame.draw.rect(screen, COL_CAR, car, 2, border_radius=12)
  pygame.draw.line(screen, (255, 95, 82), (cx, car.top + 8), (cx, car.top + 42), 4)

  b = state.blinkers
  flash = int(time.monotonic() * 2.5) % 2 == 0
  left_on = (b.left_lamp or b.left_stalk or control_cmd == "left" or control_cmd == "hazard") and flash
  right_on = (b.right_lamp or b.right_stalk or control_cmd == "right" or control_cmd == "hazard") and flash
  draw_arrow(screen, [(car.left - 34, car.top + 34), (car.left - 8, car.top + 20), (car.left - 8, car.top + 48)],
             left_on, COL_YELLOW)
  draw_arrow(screen, [(car.right + 34, car.top + 34), (car.right + 8, car.top + 20), (car.right + 8, car.top + 48)],
             right_on, COL_YELLOW)

  bs = state.blindspots
  pygame.draw.circle(screen, COL_RED if bs.rear_left else (53, 58, 65), (car.left - 18, car.bottom - 48), 13)
  pygame.draw.circle(screen, COL_RED if bs.rear_right else (53, 58, 65), (car.right + 18, car.bottom - 48), 13)
  pygame.draw.circle(screen, COL_YELLOW if bs.front_left else (53, 58, 65), (car.left - 18, car.top + 74), 11)
  pygame.draw.circle(screen, COL_YELLOW if bs.front_right else (53, 58, 65), (car.right + 18, car.top + 74), 11)

  od = state.object_distances
  sensors = [
    (cx, car.top - 30, od.front_forward_m, od.raw_detect.get("ccnc_ff", 0) > 0),
    (car.left - 32, car.top + 92, od.left_front_m, od.raw_detect.get("ccnc_lf", 0) > 0 or od.raw_detect.get("adrv_lf", 0) > 0),
    (car.right + 32, car.top + 92, od.right_front_m, od.raw_detect.get("ccnc_rf", 0) > 0 or od.raw_detect.get("adrv_rf", 0) > 0),
    (car.left - 32, car.bottom - 42, max(od.left_rear_m, od.left_corner_rear_m), od.raw_detect.get("ccnc_lr", 0) > 0 or od.raw_detect.get("adrv_lr", 0) > 0),
    (car.right + 32, car.bottom - 42, max(od.right_rear_m, od.right_corner_rear_m), od.raw_detect.get("ccnc_rr", 0) > 0 or od.raw_detect.get("adrv_rr", 0) > 0),
  ]
  for sx, sy, dist, active in sensors:
    color = distance_color(dist, active)
    radius = int(7 + clamp(8.0 - dist, 0.0, 8.0) * 1.2) if active else 7
    pygame.draw.circle(screen, color, (sx, sy), radius)


def draw_radar(screen, rect, tracks, invert_y: bool):
  draw_panel(screen, rect)
  x, y, w, h = rect
  cx = x + w // 2
  base_y = y + h - 46
  max_d = 90.0
  max_y = 18.0

  for d in range(10, 100, 10):
    py = base_y - int(d / max_d * (h - 86))
    pygame.draw.line(screen, (38, 43, 49), (x + 18, py), (x + w - 18, py), 1)
    if d % 20 == 0:
      draw_text(screen, pygame.font.SysFont("DejaVu Sans Mono", 14), x + 22, py - 9, f"{d}m", COL_MUTED)
  pygame.draw.line(screen, COL_LINE, (cx, y + 28), (cx, base_y), 1)

  pygame.draw.polygon(screen, COL_CAR, [(cx, base_y - 18), (cx - 18, base_y + 18), (cx + 18, base_y + 18)], 2)
  for t in tracks[:24]:
    y_rel = -t.y_rel if invert_y else t.y_rel
    px = cx + int(clamp(y_rel / max_y, -1.0, 1.0) * (w * 0.42))
    py = base_y - int(clamp(t.d_rel / max_d, 0.0, 1.0) * (h - 86))
    color = COL_RED if t.d_rel < 15 else COL_YELLOW if t.d_rel < 35 else COL_BLUE
    pygame.draw.circle(screen, color, (px, py), 6)
    pygame.draw.line(screen, color, (px, py), (px, py + int(clamp(-t.v_rel, -12, 12))), 2)


def draw_state(screen, font, small, state, control_enabled, control_cmd, invert_radar_y):
  w, h = screen.get_size()
  screen.fill(COL_BG)

  draw_car(screen, pygame.Rect(20, 20, 360, h - 40), state, control_cmd)
  draw_radar(screen, pygame.Rect(400, 20, w - 420, h - 220), state.radar_tracks, invert_radar_y)

  panel = pygame.Rect(400, h - 180, w - 420, 160)
  draw_panel(screen, panel)
  b = state.blinkers
  bs = state.blindspots
  od = state.object_distances

  draw_text(screen, font, 420, h - 164, "EV6 Sensors")
  draw_text(screen, small, 420, h - 132,
            f"blink stalk L/R {int(b.left_stalk)}/{int(b.right_stalk)}  lamp L/R {int(b.left_lamp)}/{int(b.right_lamp)}  hazard {int(b.hazard_candidate)}  spas {b.spas_control}")
  draw_text(screen, small, 420, h - 106,
            f"blind rear L/R {int(bs.rear_left)}/{int(bs.rear_right)}  front L/R {int(bs.front_left)}/{int(bs.front_right)}  blocked {int(bs.left_blocked)}/{int(bs.right_blocked)}")
  draw_text(screen, small, 420, h - 80,
            f"dist ff {od.front_forward_m:.1f}/{od.front_forward_alt_m:.1f}  lf/rf {od.left_front_m:.1f}/{od.right_front_m:.1f}  lr/rr {od.left_rear_m:.1f}/{od.right_rear_m:.1f}  corner {od.left_corner_rear_m:.1f}/{od.right_corner_rear_m:.1f}")
  draw_text(screen, small, 420, h - 54,
            f"radar tracks {len(state.radar_tracks)}  seen body {len(state.seen_body_addresses)}  seen aux {len(state.seen_aux_addresses)}  seen radar {len(state.seen_radar_addresses)}")
  control_color = COL_YELLOW if control_enabled and control_cmd != "off" else COL_MUTED
  draw_text(screen, small, 420, h - 28,
            f"control {'ON' if control_enabled else 'OFF'}  command {control_cmd}", control_color)


def parse_args():
  parser = argparse.ArgumentParser(description="Live EV6 sensor UI")
  parser.add_argument("--body-bus", type=int, default=0)
  parser.add_argument("--aux-bus", type=int, default=-1)
  parser.add_argument("--radar-bus", type=int, default=1)
  parser.add_argument("--radar-start", type=lambda x: int(x, 0), default=0x210)
  parser.add_argument("--radar-count", type=int, default=16)
  parser.add_argument("--can-speed", type=int, default=500)
  parser.add_argument("--data-speed", type=int, default=2000)
  parser.add_argument("--no-config", action="store_true")
  parser.add_argument("--width-px", type=int, default=1280)
  parser.add_argument("--height-px", type=int, default=800)
  parser.add_argument("--enable-blinker-control", action="store_true",
                      help="Transmit SPAS blinker commands using noOutput safety, relay closed.")
  parser.add_argument("--safety", choices=("nooutput", "unchanged"), default="nooutput",
                      help="Safety mode for blinker TX. alloutput is intentionally not supported.")
  parser.add_argument("--control-bus", type=int, default=0)
  parser.add_argument("--control-hz", type=float, default=20.0)
  parser.add_argument("--hazard-pulse-sec", type=float, default=0.15)
  parser.add_argument("--no-invert-radar-y", action="store_true",
                      help="Do not mirror radar lateral position in the UI")
  return parser.parse_args()


def main():
  args = parse_args()
  stop = False

  def handle_sigint(_sig, _frame):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGINT, handle_sigint)

  panda = Panda()
  print("Panda connected:", panda.get_serial())
  buses = {args.body_bus, args.radar_bus}
  aux_bus = None if args.aux_bus < 0 else args.aux_bus
  if aux_bus is not None:
    buses.add(aux_bus)
  control_enabled = args.enable_blinker_control
  safety_changed = False
  if control_enabled:
    buses.add(args.control_bus)
  if not args.no_config:
    configure_panda(panda, buses, args.can_speed, args.data_speed)
  if control_enabled:
    if args.safety == "nooutput":
      panda.set_safety_mode(SAFETY_NOOUTPUT, args.control_bus)
      safety_changed = True
      print(f"Blinker control enabled on bus {args.control_bus} with noOutput safety: left/right/h/0")
    else:
      print(f"Blinker control enabled on bus {args.control_bus} with unchanged safety: left/right/h/0")

  reader = EV6SensorReader(
    panda,
    body_bus=args.body_bus,
    radar_bus=args.radar_bus,
    radar_start=args.radar_start,
    radar_count=args.radar_count,
    aux_bus=aux_bus,
  )
  packer = CANPacker(BODY_DBC)

  pygame.init()
  screen = pygame.display.set_mode((args.width_px, args.height_px))
  pygame.display.set_caption("EV6 Sensor UI")
  clock = pygame.time.Clock()
  font = pygame.font.SysFont("DejaVu Sans", 24)
  small = pygame.font.SysFont("DejaVu Sans Mono", 18)

  command = "off"
  hazard_until = 0.0
  last_control = 0.0
  control_dt = 1.0 / args.control_hz
  states = deque(maxlen=3)

  try:
    while not stop:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          stop = True
        elif event.type == pygame.KEYDOWN:
          if event.key in (pygame.K_ESCAPE, pygame.K_q):
            stop = True
          elif control_enabled:
            if event.key in (pygame.K_LEFT, pygame.K_1):
              command = "left"
            elif event.key in (pygame.K_RIGHT, pygame.K_2):
              command = "right"
            elif event.key in (pygame.K_h, pygame.K_3):
              hazard_until = time.monotonic() + args.hazard_pulse_sec
            elif event.key in (pygame.K_0, pygame.K_o):
              command = "off"

      state = reader.update()
      states.append(state)

      now = time.monotonic()
      if control_enabled and now - last_control >= control_dt:
        last_control = now
        send_spas_blinker(panda, packer, "hazard" if now < hazard_until else command, args.control_bus, fd=True)

      draw_state(screen, font, small, state, control_enabled,
                 "hazard" if now < hazard_until else command, not args.no_invert_radar_y)
      pygame.display.flip()
      clock.tick(60)
  finally:
    if control_enabled:
      try:
        send_spas_blinker(panda, packer, "off", args.control_bus, fd=True)
      except Exception:
        pass
    if safety_changed:
      panda.set_safety_mode(SAFETY_SILENT, 0)
    panda.close()
    pygame.quit()
    print("Stopped.")


if __name__ == "__main__":
  main()
