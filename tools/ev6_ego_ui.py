#!/usr/bin/env python3
"""
Live EV6 ego odometry UI from Panda CAN-FD + opendbc.

Keys:
  q / ESC : quit
  r       : reset pose and trail
  c       : clear trail only
  +/-     : zoom in/out
"""

import argparse
import math
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass

import pygame


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENPILOT_ROOT = os.path.join(ROOT, "openpilot")
sys.path.insert(0, OPENPILOT_ROOT)
sys.path.insert(0, os.path.join(OPENPILOT_ROOT, "opendbc_repo"))

from opendbc.can import CANParser  # noqa: E402
from panda import Panda  # noqa: E402


DBC_NAME = "hyundai_canfd_generated"
KPH_TO_MS = 1000.0 / 3600.0


@dataclass
class EgoState:
  x: float = 0.0
  y: float = 0.0
  yaw: float = 0.0
  distance: float = 0.0
  v_mps: float = 0.0
  wheel_kph: float = 0.0
  steer_deg: float = 0.0
  road_deg: float = 0.0
  yaw_rate_dps: float = 0.0


def make_parser(bus: int) -> CANParser:
  return CANParser(DBC_NAME, [
    ("WHEEL_SPEEDS", 0),
    ("STEERING_SENSORS", 0),
    ("MDPS", 0),
  ], bus)


def configure_panda(panda: Panda, bus: int, can_speed: int, data_speed: int):
  panda.set_safety_mode(0, 0)
  panda.set_can_speed_kbps(bus, can_speed)
  panda.set_can_data_speed_kbps(bus, data_speed)
  panda.set_canfd_auto(bus, True)


def wrap_pi(angle: float) -> float:
  return (angle + math.pi) % (2.0 * math.pi) - math.pi


def vehicle_center_from_rear_axle(x: float, y: float, yaw: float, length: float,
                                  front_overhang: float, rear_overhang: float):
  rear_axle_to_center = (length * 0.5) - rear_overhang
  return (
    x + rear_axle_to_center * math.cos(yaw),
    y + rear_axle_to_center * math.sin(yaw),
  )


def step_ego(parser: CANParser, ego: EgoState, dt: float, args):
  ws = parser.vl["WHEEL_SPEEDS"]
  ss = parser.vl["STEERING_SENSORS"]
  mdps = parser.vl["MDPS"]

  wheel_kph = (
    ws["WHEEL_SPEED_1"] +
    ws["WHEEL_SPEED_2"] +
    ws["WHEEL_SPEED_3"] +
    ws["WHEEL_SPEED_4"]
  ) / 4.0

  direction = -1.0 if ws["MOVING_BACKWARD"] or ws["MOVING_BACKWARD2"] else 1.0
  v = direction * wheel_kph * KPH_TO_MS

  steer_deg = -mdps["STEERING_ANGLE_2"] if args.angle_source == "mdps" else -ss["STEERING_ANGLE"]
  if args.invert_steer:
    steer_deg *= -1.0

  road_rad = math.radians(steer_deg / args.steer_ratio)
  yaw_rate = v / args.wheelbase * math.tan(road_rad)

  yaw_mid = ego.yaw + 0.5 * yaw_rate * dt
  ego.x += v * math.cos(yaw_mid) * dt
  ego.y += v * math.sin(yaw_mid) * dt
  ego.yaw = wrap_pi(ego.yaw + yaw_rate * dt)
  ego.distance += abs(v) * dt
  ego.v_mps = v
  ego.wheel_kph = wheel_kph
  ego.steer_deg = steer_deg
  ego.road_deg = math.degrees(road_rad)
  ego.yaw_rate_dps = math.degrees(yaw_rate)


def world_to_screen(x: float, y: float, origin_px: tuple[int, int], ppm: float):
  ox, oy = origin_px
  return int(ox + x * ppm), int(oy - y * ppm)


def rotated_rect_points(cx: float, cy: float, yaw: float, length: float, width: float):
  half_l = length * 0.5
  half_w = width * 0.5
  corners = [
    (half_l, half_w),
    (half_l, -half_w),
    (-half_l, -half_w),
    (-half_l, half_w),
  ]
  c = math.cos(yaw)
  s = math.sin(yaw)
  return [(cx + px * c - py * s, cy + px * s + py * c) for px, py in corners]


def predict_path(ego: EgoState, wheelbase: float, horizon: float, dt: float):
  x = ego.x
  y = ego.y
  yaw = ego.yaw
  road_rad = math.radians(ego.road_deg)
  v = ego.v_mps
  points = []
  steps = max(1, int(horizon / dt))

  for _ in range(steps):
    yaw_rate = v / wheelbase * math.tan(road_rad)
    yaw_mid = yaw + 0.5 * yaw_rate * dt
    x += v * math.cos(yaw_mid) * dt
    y += v * math.sin(yaw_mid) * dt
    yaw = wrap_pi(yaw + yaw_rate * dt)
    points.append((x, y, yaw))

  return points


def draw_grid(screen, origin, ppm, width, height):
  bg = (18, 20, 23)
  grid = (40, 44, 49)
  axis = (75, 82, 90)
  screen.fill(bg)

  meters_per_line = 5.0
  step = max(20, int(meters_per_line * ppm))
  ox, oy = origin

  x = ox % step
  while x < width:
    pygame.draw.line(screen, grid, (x, 0), (x, height), 1)
    x += step
  y = oy % step
  while y < height:
    pygame.draw.line(screen, grid, (0, y), (width, y), 1)
    y += step

  pygame.draw.line(screen, axis, (0, oy), (width, oy), 1)
  pygame.draw.line(screen, axis, (ox, 0), (ox, height), 1)


def draw_text(screen, font, x, y, text, color=(225, 230, 235)):
  surf = font.render(text, True, color)
  screen.blit(surf, (x, y))


def draw_ui(screen, font, small_font, ego: EgoState, trail, args, ppm):
  w, h = screen.get_size()
  origin = (w // 2, h // 2)
  draw_grid(screen, origin, ppm, w, h)

  if len(trail) > 1:
    pts = [world_to_screen(x, y, origin, ppm) for x, y in trail]
    pygame.draw.lines(screen, (60, 170, 255), False, pts, 2)

  predicted = predict_path(ego, args.wheelbase, args.predict_seconds, args.predict_dt)
  if len(predicted) > 1:
    pred_pts = [world_to_screen(x, y, origin, ppm) for x, y, _ in predicted]
    pygame.draw.lines(screen, (255, 205, 80), False, pred_pts, 3)

  center_x, center_y = vehicle_center_from_rear_axle(
    ego.x, ego.y, ego.yaw, args.length, args.front_overhang, args.rear_overhang
  )
  body = rotated_rect_points(center_x, center_y, ego.yaw, args.length, args.width)
  body_pts = [world_to_screen(x, y, origin, ppm) for x, y in body]
  pygame.draw.polygon(screen, (240, 240, 235), body_pts, 2)

  nose = world_to_screen(
    center_x + (args.length * 0.5) * math.cos(ego.yaw),
    center_y + (args.length * 0.5) * math.sin(ego.yaw),
    origin,
    ppm,
  )
  center_px = world_to_screen(center_x, center_y, origin, ppm)
  pygame.draw.line(screen, (255, 95, 80), center_px, nose, 3)
  pygame.draw.circle(screen, (255, 95, 80), nose, 4)

  if predicted:
    pred_x, pred_y, pred_yaw = predicted[-1]
    pred_center_x, pred_center_y = vehicle_center_from_rear_axle(
      pred_x, pred_y, pred_yaw, args.length, args.front_overhang, args.rear_overhang
    )
    pred_body = rotated_rect_points(pred_center_x, pred_center_y, pred_yaw, args.length, args.width)
    pred_body_pts = [world_to_screen(x, y, origin, ppm) for x, y in pred_body]
    pygame.draw.polygon(screen, (255, 205, 80), pred_body_pts, 1)

  panel = pygame.Rect(16, 16, 380, 286)
  pygame.draw.rect(screen, (24, 27, 31), panel, border_radius=6)
  pygame.draw.rect(screen, (65, 70, 78), panel, 1, border_radius=6)
  draw_text(screen, font, 32, 30, "EV6 Ego Estimate")
  draw_text(screen, small_font, 32, 66, f"rear x/y      {ego.x:8.3f} / {ego.y:8.3f} m")
  draw_text(screen, small_font, 32, 92, f"center x/y    {center_x:8.3f} / {center_y:8.3f} m")
  draw_text(screen, small_font, 32, 118, f"yaw           {math.degrees(ego.yaw):8.2f} deg")
  draw_text(screen, small_font, 32, 144, f"speed         {ego.v_mps:8.2f} m/s  {ego.wheel_kph:6.2f} kph")
  draw_text(screen, small_font, 32, 170, f"steer         {ego.steer_deg:8.2f} deg")
  draw_text(screen, small_font, 32, 196, f"road angle    {ego.road_deg:8.2f} deg")
  draw_text(screen, small_font, 32, 222, f"yaw rate      {ego.yaw_rate_dps:8.2f} deg/s")
  draw_text(screen, small_font, 32, 248, f"distance      {ego.distance:8.2f} m")
  draw_text(screen, small_font, 32, 274, f"prediction    {args.predict_seconds:8.1f} s",
            (255, 215, 110))

  draw_text(screen, small_font, 16, h - 30, f"q/ESC quit  r reset  c clear  +/- zoom ({ppm:.1f} px/m)",
            (170, 178, 188))


def parse_args():
  parser = argparse.ArgumentParser(description="Live EV6 ego odometry UI")
  parser.add_argument("--bus", type=int, default=0)
  parser.add_argument("--can-speed", type=int, default=500)
  parser.add_argument("--data-speed", type=int, default=2000)
  parser.add_argument("--no-config", action="store_true")
  parser.add_argument("--width-px", type=int, default=1280)
  parser.add_argument("--height-px", type=int, default=800)
  parser.add_argument("--ppm", type=float, default=12.0, help="Pixels per meter")
  parser.add_argument("--trail-size", type=int, default=5000)
  parser.add_argument("--predict-seconds", type=float, default=3.0, help="Path prediction horizon in seconds")
  parser.add_argument("--predict-dt", type=float, default=0.05, help="Path prediction integration step in seconds")

  parser.add_argument("--length", type=float, default=4.695)
  parser.add_argument("--width", type=float, default=1.880)
  parser.add_argument("--height", type=float, default=1.550)
  parser.add_argument("--wheelbase", type=float, default=2.900)
  parser.add_argument("--front-overhang", type=float, default=0.870)
  parser.add_argument("--rear-overhang", type=float, default=0.785)
  parser.add_argument("--steer-ratio", type=float, default=16.0)
  parser.add_argument("--angle-source", choices=("sensor", "mdps"), default="sensor")
  parser.add_argument("--invert-steer", action="store_true")
  parser.add_argument("--max-dt", type=float, default=0.1)
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

  pygame.init()
  screen = pygame.display.set_mode((args.width_px, args.height_px))
  pygame.display.set_caption("EV6 Ego Estimate")
  clock = pygame.time.Clock()
  font = pygame.font.SysFont("DejaVu Sans", 24)
  small_font = pygame.font.SysFont("DejaVu Sans Mono", 20)

  ego = EgoState()
  trail = deque(maxlen=args.trail_size)
  last_t = time.monotonic()
  ppm = args.ppm

  try:
    while not stop:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          stop = True
        elif event.type == pygame.KEYDOWN:
          if event.key in (pygame.K_ESCAPE, pygame.K_q):
            stop = True
          elif event.key == pygame.K_r:
            ego = EgoState()
            trail.clear()
            last_t = time.monotonic()
          elif event.key == pygame.K_c:
            trail.clear()
          elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            ppm *= 1.2
          elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            ppm /= 1.2

      can_msgs = panda.can_recv()
      if can_msgs:
        parser.update([int(time.monotonic() * 1e9), can_msgs])

      now = time.monotonic()
      dt = min(max(0.0, now - last_t), args.max_dt)
      last_t = now
      step_ego(parser, ego, dt, args)
      trail.append((ego.x, ego.y))

      draw_ui(screen, font, small_font, ego, trail, args, ppm)
      pygame.display.flip()
      clock.tick(60)
  finally:
    panda.set_safety_mode(0, 0)
    panda.close()
    pygame.quit()
    print("Stopped.")


if __name__ == "__main__":
  main()
