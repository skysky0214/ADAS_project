#!/usr/bin/env python3
"""
EV6 SPAS blinker control UI.

Uses Panda SAFETY_NOOUTPUT with a patched whitelist for SPAS1/SPAS2 only, so the
relay stays closed and alloutput is not used.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

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
  effective_blinker_command,
  send_spas_blinker,
)


BG = (13, 15, 18)
PANEL = (24, 27, 32)
LINE = (68, 75, 84)
TEXT = (232, 236, 242)
MUTED = (145, 154, 166)
YELLOW = (255, 207, 74)
GREEN = (76, 220, 128)
RED = (255, 88, 88)
BLUE = (80, 170, 255)

COMMANDS = ("off", "left", "hazard", "right")


def parse_args():
  parser = argparse.ArgumentParser(description="EV6 blinker UI over SPAS1/SPAS2")
  parser.add_argument("--bus", type=int, default=0, help="SPAS TX bus")
  parser.add_argument("--hz", type=float, default=20.0, help="TX repeat rate")
  parser.add_argument("--hazard-hz", type=float, default=0.75, help="Hazard blink cycle rate")
  parser.add_argument("--can-speed", type=int, default=500)
  parser.add_argument("--data-speed", type=int, default=2000)
  parser.add_argument("--no-config", action="store_true", help="Do not configure Panda CAN-FD speed")
  parser.add_argument("--width", type=int, default=900)
  parser.add_argument("--height", type=int, default=420)
  return parser.parse_args()


def configure_bus(panda: Panda, bus: int, can_speed: int, data_speed: int):
  panda.set_can_speed_kbps(bus, can_speed)
  panda.set_can_data_speed_kbps(bus, data_speed)
  panda.set_canfd_auto(bus, True)


def command_from_key(key: int, current: str) -> str:
  if key in (pygame.K_LEFT, pygame.K_1, pygame.K_l):
    return "left"
  if key in (pygame.K_RIGHT, pygame.K_4, pygame.K_r):
    return "right"
  if key in (pygame.K_h, pygame.K_3):
    return "hazard"
  if key in (pygame.K_0, pygame.K_o, pygame.K_SPACE):
    return "off"
  return current


def draw_button(screen, font, rect, label: str, selected: bool, color):
  fill = color if selected else PANEL
  border = color if selected else LINE
  pygame.draw.rect(screen, fill, rect, border_radius=8)
  pygame.draw.rect(screen, border, rect, 2, border_radius=8)
  surf = font.render(label, True, BG if selected else TEXT)
  screen.blit(surf, surf.get_rect(center=rect.center))


def draw_ui(screen, title_font, font, small, command: str, bus: int, hz: float, hazard_hz: float):
  w, h = screen.get_size()
  screen.fill(BG)

  title = title_font.render("EV6 Blinker Control", True, TEXT)
  screen.blit(title, (32, 24))
  status = small.render(
    f"relay closed / noOutput / bus {bus} / tx {hz:.1f} Hz / hazard {hazard_hz:.2f} Hz",
    True,
    MUTED,
  )
  screen.blit(status, (34, 78))

  active = small.render(f"current: {command.upper()}", True, YELLOW if command != "off" else MUTED)
  screen.blit(active, (34, 108))

  gap = 18
  top = 158
  button_h = 160
  button_w = (w - 64 - gap * 3) // 4
  rects = {}
  colors = {
    "off": GREEN,
    "left": YELLOW,
    "hazard": RED,
    "right": YELLOW,
  }
  labels = {
    "off": "OFF\n0 / Space",
    "left": "LEFT\n1 / Left",
    "hazard": "HAZARD\n3 / H",
    "right": "RIGHT\n4 / Right",
  }

  for i, cmd in enumerate(COMMANDS):
    rect = pygame.Rect(32 + i * (button_w + gap), top, button_w, button_h)
    rects[cmd] = rect
    first, second = labels[cmd].split("\n")
    selected = command == cmd
    draw_button(screen, font, rect, first, selected, colors[cmd])
    hint = small.render(second, True, BG if selected else MUTED)
    screen.blit(hint, hint.get_rect(center=(rect.centerx, rect.centery + 42)))

  footer = small.render("Click buttons or use keyboard. Esc/Q exits and sends OFF.", True, MUTED)
  screen.blit(footer, (34, h - 44))
  return rects


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
    configure_bus(panda, args.bus, args.can_speed, args.data_speed)
  panda.set_safety_mode(SAFETY_NOOUTPUT, args.bus)
  print(f"Panda safety: noOutput, relay remains closed, SPAS TX bus={args.bus}")

  packer = CANPacker(BODY_DBC)
  pygame.init()
  screen = pygame.display.set_mode((args.width, args.height))
  pygame.display.set_caption("EV6 Blinker Control")
  clock = pygame.time.Clock()
  title_font = pygame.font.SysFont("DejaVu Sans", 34)
  font = pygame.font.SysFont("DejaVu Sans", 30)
  small = pygame.font.SysFont("DejaVu Sans Mono", 18)

  command = "off"
  rects = {}
  last_tx = 0.0
  tx_dt = 1.0 / args.hz

  try:
    while not stop:
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          stop = True
        elif event.type == pygame.KEYDOWN:
          if event.key in (pygame.K_ESCAPE, pygame.K_q):
            stop = True
          else:
            command = command_from_key(event.key, command)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
          for cmd, rect in rects.items():
            if rect.collidepoint(event.pos):
              command = cmd
              break

      now = time.monotonic()
      if now - last_tx >= tx_dt:
        last_tx = now
        send_spas_blinker(
          panda,
          packer,
          effective_blinker_command(command, now, args.hazard_hz),
          args.bus,
          fd=True,
        )

      rects = draw_ui(screen, title_font, font, small, command, args.bus, args.hz, args.hazard_hz)
      pygame.display.flip()
      clock.tick(60)
  finally:
    try:
      for _ in range(5):
        send_spas_blinker(panda, packer, "off", args.bus, fd=True)
        time.sleep(0.02)
    finally:
      panda.set_safety_mode(SAFETY_SILENT, 0)
      panda.close()
      pygame.quit()
      print("Stopped.")


if __name__ == "__main__":
  main()
