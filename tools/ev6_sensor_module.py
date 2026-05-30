#!/usr/bin/env python3
"""
EV6 auxiliary sensor reader using Panda + opendbc.

Read-only module for:
  - turn signals / hazard candidates
  - blind spot / corner indicators
  - ADAS/CCNC object distance candidates
  - front radar tracks

Default wiring used in this workspace:
  ECAN -> panda bus 0
  ACAN -> panda bus 1
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass, field


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENPILOT_ROOT = os.path.join(ROOT, "openpilot")
sys.path.insert(0, OPENPILOT_ROOT)
sys.path.insert(0, os.path.join(OPENPILOT_ROOT, "opendbc_repo"))

from opendbc.can import CANParser  # noqa: E402
from opendbc.can.packer import CANPacker  # noqa: E402
from panda import Panda  # noqa: E402


BODY_DBC = "hyundai_canfd_generated"
RADAR_DBC = "hyundai_canfd_radar_generated"
SAFETY_SILENT = 0
SAFETY_ALLOUTPUT = 17
SAFETY_NOOUTPUT = 19
BLINKER_CONTROL = {
  "off": 0,
  "hazard": 1,
  "left": 3,
  "right": 4,
}


@dataclass
class BlinkerState:
  left_stalk: bool = False
  right_stalk: bool = False
  left_lamp: bool = False
  right_lamp: bool = False
  hazard_lamps: bool = False
  spas_control: int = 0
  hazard_candidate: bool = False


@dataclass
class BlindspotState:
  rear_left: bool = False
  rear_right: bool = False
  front_left: bool = False
  front_right: bool = False
  left_blocked: bool = False
  right_blocked: bool = False
  collision_avoidance_active: bool = False
  raw: dict = field(default_factory=dict)


@dataclass
class ObjectDistanceState:
  front_forward_m: float = 0.0
  front_forward_alt_m: float = 0.0
  left_front_m: float = 0.0
  right_front_m: float = 0.0
  left_rear_m: float = 0.0
  right_rear_m: float = 0.0
  left_corner_rear_m: float = 0.0
  right_corner_rear_m: float = 0.0
  raw_detect: dict = field(default_factory=dict)


@dataclass
class RadarTrack:
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  source: str


@dataclass
class EV6SensorState:
  blinkers: BlinkerState = field(default_factory=BlinkerState)
  blindspots: BlindspotState = field(default_factory=BlindspotState)
  object_distances: ObjectDistanceState = field(default_factory=ObjectDistanceState)
  radar_tracks: list[RadarTrack] = field(default_factory=list)
  seen_body_addresses: set[int] = field(default_factory=set)
  seen_aux_addresses: set[int] = field(default_factory=set)
  seen_radar_addresses: set[int] = field(default_factory=set)


def make_body_parser(bus: int) -> CANParser:
  optional = math.nan
  msgs = [
    ("BLINKER_STALKS", optional),
    ("BLINKERS", optional),
    ("SPAS2", optional),
    ("ADRV_0x161", optional),
    ("CCNC_0x162", optional),
    ("ADRV_0x1ea", optional),
    ("BLINDSPOTS_REAR_CORNERS", optional),
    ("BLINDSPOTS_FRONT_CORNER_1", optional),
    ("BLINDSPOTS_FRONT_CORNER_2", optional),
  ]
  return CANParser(BODY_DBC, msgs, bus)


def make_aux_parser(bus: int) -> CANParser:
  optional = math.nan
  msgs = [
    ("ADRV_0x161", optional),
    ("CCNC_0x162", optional),
    ("ADRV_0x1ea", optional),
  ]
  return CANParser(BODY_DBC, msgs, bus)


def make_radar_parser(bus: int, start_addr: int, count: int) -> CANParser:
  msgs = [(f"RADAR_TRACK_{addr:x}", math.nan) for addr in range(start_addr, start_addr + count)]
  return CANParser(RADAR_DBC, msgs, bus)


def configure_panda(panda: Panda, buses: set[int], can_speed: int, data_speed: int):
  panda.set_safety_mode(SAFETY_SILENT, 0)
  for bus in buses:
    panda.set_can_speed_kbps(bus, can_speed)
    panda.set_can_data_speed_kbps(bus, data_speed)
    panda.set_canfd_auto(bus, True)


def normalize_rx_messages(can_msgs):
  # Panda marks returned TX frames as bus + 128 and rejected TX as bus + 192.
  # Sensor readers should only parse actual RX traffic.
  return [(addr, dat, bus) for addr, dat, bus in can_msgs if bus < 128]


def build_spas_blinker_msgs(packer: CANPacker, command: str, bus: int):
  if command not in BLINKER_CONTROL:
    raise ValueError(f"unsupported blinker command: {command}")
  return [
    packer.make_can_msg("SPAS1", bus, {}),
    packer.make_can_msg("SPAS2", bus, {"BLINKER_CONTROL": BLINKER_CONTROL[command]}),
  ]


def send_spas_blinker(panda: Panda, packer: CANPacker, command: str, bus: int, fd: bool = True):
  msgs = build_spas_blinker_msgs(packer, command, bus)
  panda.can_send_many([[addr, dat, src] for addr, dat, src in msgs], fd=fd)


def effective_blinker_command(command: str, now: float | None = None, hazard_hz: float = 0.75) -> str:
  if command != "hazard":
    return command
  if now is None:
    now = time.monotonic()
  return "hazard" if (now * hazard_hz) % 1.0 < 0.5 else "off"


def get_blinkers(body: CANParser) -> BlinkerState:
  stalks = body.vl["BLINKER_STALKS"]
  lamps = body.vl["BLINKERS"]
  spas2 = body.vl["SPAS2"]

  left_lamp = bool(lamps["LEFT_LAMP"] or lamps["LEFT_LAMP_ALT"])
  right_lamp = bool(lamps["RIGHT_LAMP"] or lamps["RIGHT_LAMP_ALT"])
  spas_control = int(spas2["BLINKER_CONTROL"])

  return BlinkerState(
    left_stalk=bool(stalks["LEFT_BLINKER"] or lamps["LEFT_STALK"]),
    right_stalk=bool(stalks["RIGHT_BLINKER"] or lamps["RIGHT_STALK"]),
    left_lamp=left_lamp,
    right_lamp=right_lamp,
    hazard_lamps=left_lamp and right_lamp,
    spas_control=spas_control,
    hazard_candidate=spas_control == 1 or (left_lamp and right_lamp),
  )


def get_blindspots(body: CANParser) -> BlindspotState:
  rear = body.vl["BLINDSPOTS_REAR_CORNERS"]
  fc1 = body.vl["BLINDSPOTS_FRONT_CORNER_1"]
  fc2 = body.vl["BLINDSPOTS_FRONT_CORNER_2"]

  rear_left = bool(rear["FL_INDICATOR"] or rear["INDICATOR_LEFT_TWO"] or
                   rear["INDICATOR_LEFT_THREE"] or rear["INDICATOR_LEFT_FOUR"])
  rear_right = bool(rear["FR_INDICATOR"] or rear["INDICATOR_RIGHT_TWO"] or
                    rear["INDICATOR_RIGHT_THREE"] or rear["INDICATOR_RIGHT_FOUR"])

  return BlindspotState(
    rear_left=rear_left,
    rear_right=rear_right,
    front_left=bool(fc2["LEFT_BSD"]),
    front_right=bool(fc2["RIGHT_BSD"]),
    left_blocked=bool(rear["LEFT_BLOCKED"]),
    right_blocked=bool(rear["RIGHT_BLOCKED"]),
    collision_avoidance_active=bool(rear["COLLISION_AVOIDANCE_ACTIVE"]),
    raw={
      "rear": dict(rear),
      "front_corner_1": dict(fc1),
      "front_corner_2": dict(fc2),
    },
  )


def get_object_distances(body: CANParser) -> ObjectDistanceState:
  ccnc = body.vl["CCNC_0x162"]
  adrv = body.vl["ADRV_0x1ea"]

  return ObjectDistanceState(
    front_forward_m=ccnc["FF_DISTANCE"],
    front_forward_alt_m=ccnc["FF_DISTANCE_ALT"],
    left_front_m=max(ccnc["LF_DETECT_DISTANCE"], adrv["LF_DETECT_DISTANCE"]),
    right_front_m=max(ccnc["RF_DETECT_DISTANCE"], adrv["RF_DETECT_DISTANCE"]),
    left_rear_m=ccnc["LR_DETECT_DISTANCE"],
    right_rear_m=ccnc["RR_DETECT_DISTANCE"],
    left_corner_rear_m=adrv["CORNER_LR_DIST"],
    right_corner_rear_m=adrv["CORNER_RR_DIST"],
    raw_detect={
      "ccnc_ff": ccnc["FF_DETECT"],
      "ccnc_lf": ccnc["LF_DETECT"],
      "ccnc_rf": ccnc["RF_DETECT"],
      "ccnc_lr": ccnc["LR_DETECT"],
      "ccnc_rr": ccnc["RR_DETECT"],
      "adrv_lf": adrv["LF_DETECT"],
      "adrv_rf": adrv["RF_DETECT"],
      "adrv_lr": adrv["CORNER_LR_DETECT"],
      "adrv_rr": adrv["CORNER_RR_DETECT"],
    },
  )


def merge_object_distances(*states: ObjectDistanceState) -> ObjectDistanceState:
  merged_detect = {}
  for state in states:
    for key, value in state.raw_detect.items():
      merged_detect[key] = max(merged_detect.get(key, 0), value)

  return ObjectDistanceState(
    front_forward_m=max((s.front_forward_m for s in states), default=0.0),
    front_forward_alt_m=max((s.front_forward_alt_m for s in states), default=0.0),
    left_front_m=max((s.left_front_m for s in states), default=0.0),
    right_front_m=max((s.right_front_m for s in states), default=0.0),
    left_rear_m=max((s.left_rear_m for s in states), default=0.0),
    right_rear_m=max((s.right_rear_m for s in states), default=0.0),
    left_corner_rear_m=max((s.left_corner_rear_m for s in states), default=0.0),
    right_corner_rear_m=max((s.right_corner_rear_m for s in states), default=0.0),
    raw_detect=merged_detect,
  )


def get_radar_tracks(radar: CANParser, start_addr: int, count: int,
                     valid_threshold: int = 10) -> list[RadarTrack]:
  tracks: list[RadarTrack] = []
  track_id = 0
  for addr in range(start_addr, start_addr + count):
    msg_name = f"RADAR_TRACK_{addr:x}"
    msg = radar.vl[msg_name]

    if "VALID_CNT1" in msg:
      if msg["VALID_CNT1"] > valid_threshold:
        tracks.append(RadarTrack(
          track_id=track_id,
          d_rel=msg["LONG_DIST1"],
          y_rel=msg["LAT_DIST1"],
          v_rel=msg["REL_SPEED1"],
          a_rel=msg["REL_ACCEL1"],
          yv_rel=msg["LAT_SPEED1"],
          source=f"{msg_name}:1",
        ))
      track_id += 1
      if msg["VALID_CNT2"] > valid_threshold:
        tracks.append(RadarTrack(
          track_id=track_id,
          d_rel=msg["LONG_DIST2"],
          y_rel=msg["LAT_DIST2"],
          v_rel=msg["REL_SPEED2"],
          a_rel=msg["REL_ACCEL2"],
          yv_rel=msg["LAT_SPEED2"],
          source=f"{msg_name}:2",
        ))
      track_id += 1
    else:
      if msg["VALID_CNT"] > valid_threshold:
        tracks.append(RadarTrack(
          track_id=track_id,
          d_rel=msg["LONG_DIST"],
          y_rel=msg["LAT_DIST"],
          v_rel=msg["REL_SPEED"],
          a_rel=msg["REL_ACCEL"],
          yv_rel=msg["LAT_SPEED"],
          source=msg_name,
        ))
      track_id += 1

  tracks.sort(key=lambda t: t.d_rel)
  return tracks


class EV6SensorReader:
  def __init__(self, panda: Panda, body_bus: int = 0, radar_bus: int = 1,
               radar_start: int = 0x210, radar_count: int = 16, aux_bus: int | None = None):
    self.panda = panda
    self.body_bus = body_bus
    self.radar_bus = radar_bus
    self.aux_bus = aux_bus
    self.radar_start = radar_start
    self.radar_count = radar_count
    self.body_parser = make_body_parser(body_bus)
    self.aux_parser = make_aux_parser(aux_bus) if aux_bus is not None else None
    self.radar_parser = make_radar_parser(radar_bus, radar_start, radar_count)

  def update(self) -> EV6SensorState:
    can_msgs = normalize_rx_messages(self.panda.can_recv())
    now_nanos = int(time.monotonic() * 1e9)
    if can_msgs:
      update = [now_nanos, can_msgs]
      self.body_parser.update(update)
      if self.aux_parser is not None:
        self.aux_parser.update(update)
      self.radar_parser.update(update)

    distances = get_object_distances(self.body_parser)
    aux_seen = set()
    if self.aux_parser is not None:
      aux_seen = set(self.aux_parser.seen_addresses)
      distances = merge_object_distances(distances, get_object_distances(self.aux_parser))

    state = EV6SensorState(
      blinkers=get_blinkers(self.body_parser),
      blindspots=get_blindspots(self.body_parser),
      object_distances=distances,
      radar_tracks=get_radar_tracks(self.radar_parser, self.radar_start, self.radar_count),
      seen_body_addresses=set(self.body_parser.seen_addresses),
      seen_aux_addresses=aux_seen,
      seen_radar_addresses=set(self.radar_parser.seen_addresses),
    )
    return state


def fmt_bool(v: bool) -> str:
  return "1" if v else "0"


def format_state(state: EV6SensorState, max_tracks: int) -> str:
  b = state.blinkers
  bs = state.blindspots
  od = state.object_distances
  tracks = state.radar_tracks[:max_tracks]
  track_s = " ".join(
    f"#{t.track_id}:d={t.d_rel:.1f} y={t.y_rel:.1f} v={t.v_rel:.1f}" for t in tracks
  ) or "none"

  return (
    f"blink stalk L/R={fmt_bool(b.left_stalk)}/{fmt_bool(b.right_stalk)} "
    f"lamp L/R={fmt_bool(b.left_lamp)}/{fmt_bool(b.right_lamp)} "
    f"hazard={fmt_bool(b.hazard_candidate)} spas={b.spas_control} | "
    f"blind rear L/R={fmt_bool(bs.rear_left)}/{fmt_bool(bs.rear_right)} "
    f"front L/R={fmt_bool(bs.front_left)}/{fmt_bool(bs.front_right)} "
    f"blocked L/R={fmt_bool(bs.left_blocked)}/{fmt_bool(bs.right_blocked)} | "
    f"dist ff={od.front_forward_m:.1f}/{od.front_forward_alt_m:.1f} "
    f"lf/rf={od.left_front_m:.1f}/{od.right_front_m:.1f} "
    f"lr/rr={od.left_rear_m:.1f}/{od.right_rear_m:.1f} "
    f"corner_lr/rr={od.left_corner_rear_m:.1f}/{od.right_corner_rear_m:.1f} | "
    f"radar tracks={len(state.radar_tracks)} {track_s}"
  )


def parse_args():
  parser = argparse.ArgumentParser(description="Read EV6 auxiliary ADAS sensors")
  parser.add_argument("--body-bus", type=int, default=0, help="Bus for blinkers/blindspots/ADAS display messages")
  parser.add_argument("--aux-bus", type=int, default=-1, help="Bus for auxiliary ADRV/CCNC distance messages, use -1 to disable")
  parser.add_argument("--radar-bus", type=int, default=1, help="Bus for radar tracks, typically ACAN")
  parser.add_argument("--radar-start", type=lambda x: int(x, 0), default=0x210, help="Radar start address")
  parser.add_argument("--radar-count", type=int, default=16, help="Number of radar track messages")
  parser.add_argument("--can-speed", type=int, default=500)
  parser.add_argument("--data-speed", type=int, default=2000)
  parser.add_argument("--print-hz", type=float, default=10.0)
  parser.add_argument("--max-tracks", type=int, default=6)
  parser.add_argument("--no-config", action="store_true")
  parser.add_argument("--show-seen", action="store_true", help="Print seen addresses each cycle")
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
  aux_bus = None if args.aux_bus < 0 else args.aux_bus
  if not args.no_config:
    buses = {args.body_bus, args.radar_bus}
    if aux_bus is not None:
      buses.add(aux_bus)
    configure_panda(panda, buses, args.can_speed, args.data_speed)
    print(
      f"Configured CAN-FD: nominal={args.can_speed} kbps data={args.data_speed} kbps "
      f"body_bus={args.body_bus} aux_bus={aux_bus} radar_bus={args.radar_bus}"
    )

  reader = EV6SensorReader(
    panda,
    body_bus=args.body_bus,
    radar_bus=args.radar_bus,
    radar_start=args.radar_start,
    radar_count=args.radar_count,
    aux_bus=aux_bus,
  )
  print_dt = 1.0 / args.print_hz
  next_print = 0.0

  try:
    while not stop:
      state = reader.update()
      now = time.monotonic()
      if now >= next_print:
        next_print = now + print_dt
        print(format_state(state, args.max_tracks))
        if args.show_seen:
          body_seen = " ".join(f"0x{x:x}" for x in sorted(state.seen_body_addresses))
          aux_seen = " ".join(f"0x{x:x}" for x in sorted(state.seen_aux_addresses))
          radar_seen = " ".join(f"0x{x:x}" for x in sorted(state.seen_radar_addresses))
          print(f"  seen body: {body_seen}")
          print(f"  seen aux: {aux_seen}")
          print(f"  seen radar: {radar_seen}")
      time.sleep(0.001)
  finally:
    panda.set_safety_mode(SAFETY_SILENT, 0)
    panda.close()
    print("Stopped.")


if __name__ == "__main__":
  main()
