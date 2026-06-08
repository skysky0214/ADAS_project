#!/usr/bin/env python3
"""EV6 joystick control harness using openpilot's Hyundai stack.

This script reuses the stock CarInterface + CarController to translate
joystick inputs into LKAS/SCC CAN-FD commands for testing without the
full comma device.  It expects a Red Panda connected both to the vehicle
and this PC via USB.

Requirements:
  * `pip install inputs` for joystick support.
  * Panda python package dependencies are already part of this repo.

Usage example:
  ./tools/joystick_ev6_control.py --car KIA_EV6 --start-enabled \
      --set-speed-kph 40 --enable-button BTN_SOUTH
"""

import argparse
import json
import os
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "openpilot"))
sys.path.insert(0, str(REPO_ROOT / "openpilot" / "opendbc_repo"))
sys.path.insert(0, str(REPO_ROOT))

from cereal import car
from panda import Panda
from opendbc.car import structs
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper

# Panda 버스 ↔ 차량 버스 매핑 (ACAN=1, ECAN=0)
HW_TO_LOGICAL_BUS = {
  0: 1,  # Panda bus 0 -> logical bus 1 (ACAN)
  1: 0,  # Panda bus 1 -> logical bus 0 (ECAN)
  2: 2,
}
LOGICAL_TO_HW_BUS = {v: k for k, v in HW_TO_LOGICAL_BUS.items()}

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert


def require_inputs_getter():
  try:
    from inputs import get_gamepad  # type: ignore
  except ImportError as exc:
    raise SystemExit("Missing dependency 'inputs'. Install with `pip install inputs`." ) from exc
  return get_gamepad


@dataclass
class AxisConfig:
  code: str
  minimum: float
  maximum: float
  centered: bool


@dataclass
class JoystickSnapshot:
  steer: float
  throttle: float
  brake: float
  enabled: bool


@dataclass
class CruiseControlShim:
  cancel: bool = False
  resume: bool = False
  override: bool = False


@dataclass
class HudControlShim:
  setSpeed: float = 0.0
  leadDistanceBars: int = 0
  leadVisible: bool = False
  leadDPath: float = 0.0
  leadDistance: float = 0.0
  leadRelSpeed: float = 0.0
  leftLaneVisible: bool = False
  rightLaneVisible: bool = False
  activeCarrot: int = 0
  modelDesire: int = 0
  atcDistance: float = 1000.0
  leadRadar: int = 0
  leadDistance2: float = 0.0
  leadLeftDist: float = 0.0
  leadRightDist: float = 0.0
  leadLeftLat: float = 0.0
  leadRightLat: float = 0.0
  leadLeftDist2: float = 0.0
  leadRightDist2: float = 0.0
  leadLeftLat2: float = 0.0
  leadRightLat2: float = 0.0
  leftLaneDepart: bool = False
  rightLaneDepart: bool = False
  speedVisible: bool = True
  lanesVisible: bool = True
  visualAlert: int = VisualAlert.none


@dataclass
class ActuatorsShim:
  torque: float = 0.0
  steeringAngleDeg: float = 0.0
  curvature: float = 0.0
  accel: float = 0.0
  aTarget: float = 0.0
  jerk: float = 0.0
  longControlState: LongCtrlState = LongCtrlState.off
  gas: float = 0.0
  brake: float = 0.0
  torqueOutputCan: float = 0.0
  speed: float = 0.0

  def as_builder(self):
    return self


@dataclass
class CarControlShim:
  enabled: bool
  latActive: bool
  longActive: bool
  actuators: ActuatorsShim
  hudControl: HudControlShim
  cruiseControl: CruiseControlShim


class JoystickReader:
  def __init__(self, steer_cfg: AxisConfig, throttle_cfg: AxisConfig, brake_cfg: AxisConfig,
               deadzone: float, enable_button: Optional[str], start_enabled: bool):
    self._steer_cfg = steer_cfg
    self._throttle_cfg = throttle_cfg
    self._brake_cfg = brake_cfg
    self._deadzone = deadzone
    self._enable_button = enable_button
    self._state = JoystickSnapshot(steer=0.0, throttle=0.0, brake=0.0, enabled=start_enabled)
    self._lock = threading.Lock()
    self._get_gamepad = require_inputs_getter()
    self._stop = threading.Event()
    self._thread = threading.Thread(target=self._run, name="joystick_reader", daemon=True)
    self._thread.start()

  def stop(self) -> None:
    self._stop.set()
    self._thread.join(timeout=1.0)

  def snapshot(self) -> JoystickSnapshot:
    with self._lock:
      return JoystickSnapshot(**self._state.__dict__)

  def _run(self) -> None:
    while not self._stop.is_set():
      try:
        events = self._get_gamepad()
      except OSError:
        time.sleep(0.1)
        continue

      for evt in events:
        if evt.ev_type == 'Absolute':
          self._handle_axis(evt.code, evt.state)
        elif evt.ev_type == 'Key' and self._enable_button and evt.code == self._enable_button and evt.state == 1:
          with self._lock:
            self._state.enabled = not self._state.enabled
            status = "ENABLED" if self._state.enabled else "DISABLED"
            print(f"[joystick] toggle via {self._enable_button}: {status}")

  def _handle_axis(self, code: str, raw_value: float) -> None:
    updated = False
    with self._lock:
      if code == self._steer_cfg.code:
        self._state.steer = self._normalize_signed(raw_value, self._steer_cfg)
        updated = True
      elif code == self._throttle_cfg.code:
        self._state.throttle = self._normalize_unsigned(raw_value, self._throttle_cfg)
        updated = True
      elif code == self._brake_cfg.code:
        self._state.brake = self._normalize_unsigned(raw_value, self._brake_cfg)
        updated = True
    if updated and code == self._steer_cfg.code:
      pass

  def _normalize_signed(self, value: float, cfg: AxisConfig) -> float:
    center = 0.5 * (cfg.maximum + cfg.minimum)
    amplitude = max(cfg.maximum - center, center - cfg.minimum)
    if amplitude <= 0.0:
      return 0.0
    norm = (value - center) / amplitude if cfg.centered else value / amplitude
    if abs(norm) < self._deadzone:
      return 0.0
    return float(max(-1.0, min(1.0, norm)))

  def _normalize_unsigned(self, value: float, cfg: AxisConfig) -> float:
    span = cfg.maximum - cfg.minimum
    if span <= 0.0:
      return 0.0
    return float(max(0.0, min(1.0, (value - cfg.minimum) / span)))


def collect_fingerprint(panda: Panda, duration_s: float, buses: int = 3) -> Dict[int, Dict[int, int]]:
  print(f"Collecting CAN fingerprint for {duration_s:.1f}s...")
  start = time.monotonic()
  fp: Dict[int, Dict[int, int]] = defaultdict(dict)
  while time.monotonic() - start < duration_s:
    msgs = panda.can_recv()
    for address, dat, src in msgs:
      if 0 <= src < buses:
        fp[src][address] = len(dat)
  normalized = {bus: dict(fp.get(bus, {})) for bus in range(buses)}
  print("Fingerprint capture complete.")
  return normalized


def panda_msgs_to_can_list(msgs: List[tuple[int, bytes, int]]) -> List[tuple[int, List[tuple[int, bytes, int]]]]:
  if not msgs:
    return []
  # openpilot's CAN stack expects monotonic timestamps (sec_since_boot).
  now = time.monotonic_ns()
  frames = []
  for address, dat, src in msgs:
    logical_bus = HW_TO_LOGICAL_BUS.get(src, src)
    frames.append((address, dat, logical_bus))
  return [(now, frames)]


def configure_panda(panda: Panda, cp: structs.CarParams, can_kbps: int, data_kbps: int, enable_canfd: bool) -> None:
  hw_type = panda.get_type()
  if enable_canfd:
    print(f"Configuring CAN-FD speeds: {can_kbps} / {data_kbps} kbps")
  else:
    print(f"Configuring CAN speeds: {can_kbps} kbps")
  panda.set_safety_mode(car.CarParams.SafetyModel.silent)
  for bus in range(3):
    panda.set_can_speed_kbps(bus, can_kbps)
    if hw_type in (Panda.HW_TYPE_RED_PANDA, Panda.HW_TYPE_RED_PANDA_V2) and enable_canfd:
      panda.set_can_data_speed_kbps(bus, data_kbps)
    elif enable_canfd:
      panda.set_can_data_speed_kbps(bus, 10)

  if not cp.safetyConfigs:
    raise RuntimeError("CarParams did not include safety configuration")
  safety = cp.safetyConfigs[0]
  safety_model = safety.safetyModel
  model_val = safety_model.raw if hasattr(safety_model, "raw") else int(safety_model)
  panda.set_safety_mode(model_val, int(safety.safetyParam))
  print(f"Panda safety set to {model_val} / {int(safety.safetyParam)}")


def build_car_control(snapshot: JoystickSnapshot, accel_max: float, decel_max: float,
                      set_speed: float, distance_bars: int) -> CarControlShim:
  actuators = ActuatorsShim()
  actuators.torque = snapshot.steer
  actuators.steeringAngleDeg = 0.0
  actuators.curvature = 0.0
  accel = snapshot.throttle * accel_max - snapshot.brake * decel_max
  actuators.accel = accel
  actuators.aTarget = accel
  actuators.jerk = 0.0
  actuators.longControlState = LongCtrlState.pid if snapshot.enabled else LongCtrlState.off

  hud = HudControlShim()
  hud.setSpeed = set_speed
  hud.leadDistanceBars = distance_bars
  hud.leftLaneVisible = True
  hud.rightLaneVisible = True
  hud.visualAlert = VisualAlert.none

  cruise = CruiseControlShim()

  return CarControlShim(
    enabled=snapshot.enabled,
    latActive=snapshot.enabled,
    longActive=snapshot.enabled,
    actuators=actuators,
    hudControl=hud,
    cruiseControl=cruise,
  )


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Joystick-driven EV6 control using openpilot modules")
  parser.add_argument('--car', default='KIA_EV6', help='Hyundai CAR enum name (default: KIA_EV6)')
  parser.add_argument('--fingerprint-time', type=float, default=3.0, help='Seconds to sniff CAN for fingerprinting')
  parser.add_argument('--set-speed-kph', type=float, default=40.0, help='Virtual cruise set speed shown on HUD')
  parser.add_argument('--distance-bars', type=int, default=2, help='HUD lead distance bars (1-4)')
  parser.add_argument('--camera-scc', type=int, default=0, choices=(0, 1, 2, 3), help='Value for HyundaiCameraSCC param')
  parser.add_argument('--steer-axis', default='ABS_X', help='Joystick axis code for steering')
  parser.add_argument('--steer-min', type=float, default=-32768, help='Minimum raw value for steering axis')
  parser.add_argument('--steer-max', type=float, default=32767, help='Maximum raw value for steering axis')
  parser.add_argument('--throttle-axis', default='ABS_RZ', help='Axis code for throttle')
  parser.add_argument('--throttle-min', type=float, default=0, help='Minimum throttle raw value')
  parser.add_argument('--throttle-max', type=float, default=255, help='Maximum throttle raw value')
  parser.add_argument('--brake-axis', default='ABS_Z', help='Axis code for brake')
  parser.add_argument('--brake-min', type=float, default=0, help='Minimum brake raw value')
  parser.add_argument('--brake-max', type=float, default=255, help='Maximum brake raw value')
  parser.add_argument('--deadzone', type=float, default=0.05, help='Steering deadzone (normalized)')
  parser.add_argument('--max-accel', type=float, default=2.5, help='Maximum accel command (m/s^2)')
  parser.add_argument('--max-decel', type=float, default=3.5, help='Maximum decel command (m/s^2) when brake fully pressed')
  parser.add_argument('--enable-button', default='BTN_SOUTH', help='Button code to toggle enable (set to NONE to ignore)')
  parser.add_argument('--start-enabled', action='store_true', help='Start with control enabled')
  parser.add_argument('--loop-rate', type=float, default=100.0, help='Control loop rate in Hz')
  parser.add_argument('--can-speed', type=int, default=500, help='CAN arbitration speed (kbps)')
  parser.add_argument('--data-speed', type=int, default=2000, help='CAN FD data speed (kbps)')
  parser.add_argument('--disable-canfd', action='store_true', help='Send classic CAN frames only (no CAN-FD data speed)')
  parser.add_argument('--log-can', type=str, default=None,
                      help='Optional path to log received CAN frames (jsonl)')
  return parser.parse_args()


def main() -> None:
  args = parse_args()

  car_name = args.car
  if not hasattr(CAR, car_name):
    raise SystemExit(f"Unknown CAR enum '{car_name}'")
  candidate = getattr(CAR, car_name)

  params = Params()
  params.put_int("HyundaiCameraSCC", int(args.camera_scc))

  enable_button = None if args.enable_button.upper() == 'NONE' else args.enable_button
  joystick = JoystickReader(
    steer_cfg=AxisConfig(args.steer_axis, args.steer_min, args.steer_max, centered=True),
    throttle_cfg=AxisConfig(args.throttle_axis, args.throttle_min, args.throttle_max, centered=False),
    brake_cfg=AxisConfig(args.brake_axis, args.brake_min, args.brake_max, centered=False),
    deadzone=args.deadzone,
    enable_button=enable_button,
    start_enabled=args.start_enabled,
  )

  set_speed = args.set_speed_kph / 3.6
  print(f"Target HUD speed: {args.set_speed_kph:.1f} km/h")

  panda = Panda()
  log_file = open(args.log_can, "w") if args.log_can else None
  try:
    fingerprint = collect_fingerprint(panda, args.fingerprint_time)
    params.put("FingerPrints", str(fingerprint))
    CP = CarInterface.get_params(candidate, fingerprint, [], True, False, False)

    if CP.flags & HyundaiFlags.CANFD_HDA2:
      current_cam = params.get_int("HyundaiCameraSCC")
      if current_cam != 0:
        print("Detected HDA2 platform, forcing HyundaiCameraSCC=0 for SCC2 compatibility.")
        params.put_int("HyundaiCameraSCC", 0)
        CP = CarInterface.get_params(candidate, fingerprint, [], True, False, False)

    CI = CarInterface(CP)

    configure_panda(panda, CP, args.can_speed, args.data_speed, not args.disable_canfd)

    rk = Ratekeeper(args.loop_rate)
    print("Entering control loop. Use the configured joystick button to toggle enable.")

    while True:
      msgs = panda.can_recv()
      can_list = panda_msgs_to_can_list(msgs)
      if log_file and msgs:
        log_time = time.time()
        for address, dat, src in msgs:
          log_file.write(json.dumps({
            "ts": log_time,
            "address": address,
            "bytes": dat.hex(),
            "bus": src,
          }) + "\n")
      if can_list:
        CI.update(can_list)

      snapshot = joystick.snapshot()
      cc = build_car_control(snapshot, args.max_accel, args.max_decel, set_speed, args.distance_bars)
      _, can_sends = CI.apply(cc)
      if can_sends:
        def _unwrap_msg(msg):
          if hasattr(msg, "address"):
            return [msg.address, msg.dat, msg.src]
          return [msg[0], msg[1], msg[2]]
        unwrapped = []
        for msg in can_sends:
          addr, dat, bus = _unwrap_msg(msg)
          hw_bus = LOGICAL_TO_HW_BUS.get(bus, bus)
          unwrapped.append([addr, dat, hw_bus])
        panda.can_send_many(unwrapped, fd=not args.disable_canfd)

      rk.keep_time()
  except KeyboardInterrupt:
    print("Stopping joystick control.")
  finally:
    joystick.stop()
    try:
      panda.set_safety_mode(int(car.CarParams.SafetyModel.silent), 0)
    except Exception:
      pass
    panda.close()
    if log_file:
      log_file.close()


if __name__ == "__main__":
  os.environ.setdefault("OPENPILOT_PREFIX", "_pc")
  main()
