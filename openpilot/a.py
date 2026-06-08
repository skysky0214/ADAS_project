#!/usr/bin/env python3
"""
간단한 EV6/HDA2 스티어링 테스트 스크립트.

 - 조이스틱 (pygame) 입력을 받아 LKAS/LFA 메시지를 직접 생성해 보냅니다.
 - Panda는 alloutput 모드로 설정하여 안전훅 간섭 없이 송신합니다.
 - 필요한 최소 메시지(LKAS, LFA, 선택적으로 LFAHDA_CLUSTER)를 순환 전송합니다.
"""

import argparse
import signal
import threading
import time
from dataclasses import dataclass

import pygame

from panda import Panda
from opendbc.can.packer import CANPacker

# 기본 버스 매핑 (Red Panda 기준)
DEFAULT_ACAN = 1  # 차량측 A-CAN (LKAS)
DEFAULT_ECAN = 0  # 차량측 E-CAN (LFA/LFAHDA/SCC_CONTROL)

STEER_MAX = 400


@dataclass
class JoystickState:
  steer: float = 0.0  # -1.0 ~ 1.0
  longitudinal: float = 0.0  # -1.0 ~ 1.0 (가감속)
  accel: float = 0.0  # 0.0 ~ 1.0
  brake: float = 0.0  # 0.0 ~ 1.0
  enabled: bool = True


class JoystickReader:
  def __init__(self, deadzone: float, max_angle_deg: float, accel_axis: int, brake_axis: int):
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
      raise RuntimeError("조이스틱을 찾을 수 없습니다.")

    self.deadzone = deadzone
    self.max_angle = max_angle_deg
    self.accel_axis = accel_axis
    self.brake_axis = brake_axis
    self.joy = pygame.joystick.Joystick(0)
    self.joy.init()

    self.state = JoystickState()
    self._lock = threading.Lock()
    self._stop = threading.Event()
    self._thread = threading.Thread(target=self._run, daemon=True)
    self._thread.start()

  def stop(self):
    self._stop.set()
    self._thread.join(timeout=1.0)

  def snapshot(self) -> JoystickState:
    with self._lock:
      return JoystickState(
        self.state.steer,
        self.state.longitudinal,
        self.state.accel,
        self.state.brake,
        self.state.enabled,
      )

  def _trigger_to_norm(self, val: float) -> float:
    # 트리거는 보통 -1.0(미입력) ~ 1.0(완전 입력)로 들어옴
    return max(0.0, min(1.0, (val + 1.0) * 0.5))

  def _run(self):
    print(f"조이스틱 연결: {self.joy.get_name()}")
    while not self._stop.is_set():
      pygame.event.pump()
      axis = self.joy.get_axis(0) if self.joy.get_numaxes() > 0 else 0.0
      accel_axis = self.joy.get_axis(self.accel_axis) if self.joy.get_numaxes() > self.accel_axis else -1.0
      brake_axis = self.joy.get_axis(self.brake_axis) if self.joy.get_numaxes() > self.brake_axis else -1.0
      button_a = self.joy.get_button(0) if self.joy.get_numbuttons() > 0 else False
      button_b = self.joy.get_button(1) if self.joy.get_numbuttons() > 1 else False

      accel_norm = self._trigger_to_norm(accel_axis)
      brake_norm = self._trigger_to_norm(brake_axis)

      with self._lock:
        if abs(axis) < self.deadzone:
          self.state.steer = 0.0
        else:
          self.state.steer = -axis  # 왼쪽/오른쪽 반전
        if accel_norm < self.deadzone and brake_norm < self.deadzone:
          self.state.longitudinal = 0.0
        else:
          # 가속(+) - 감속(-)
          self.state.longitudinal = accel_norm - brake_norm
        self.state.accel = accel_norm
        self.state.brake = brake_norm
        if button_a:
          self.state.enabled = not self.state.enabled
          status = "ENABLED" if self.state.enabled else "DISABLED"
          print(f"[Joystick] A 버튼 → {status}")
        if button_b:
          print("[Joystick] B 버튼 → 종료")
          self._stop.set()
          break

      time.sleep(0.02)


def build_lkas_msg(packer: CANPacker, torque: int, enabled: bool, bus: int):
  values = {
    "LKA_MODE": 2,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": torque,
    "VALUE104": 3 if enabled else 100,
    "STEER_REQ": 1 if enabled else 0,
    "HAS_LANE_SAFETY": 0,
    "VALUE63": 0,
    "VALUE64": 0,
  }
  return packer.make_can_msg("LKAS", bus, values)


def build_lfa_msg(packer: CANPacker, torque: int, enabled: bool, counter: int, bus: int):
  values = {
    "CHECKSUM": 0,
    "COUNTER": counter % 256,
    "LKA_MODE": 2 if enabled else 0,
    "LKA_ACTIVE": 2 if enabled else 0,
    "LKA_ICON": 2 if enabled else 1,
    "TORQUE_REQUEST": torque,
    "STEER_REQ": 1 if enabled else 0,
    "LFA_BUTTON": 0,
    "VALUE63": 0,
    "VALUE64": 0,
    "NEW_SIGNAL_1": 0,
    "LKAS_ANGLE_ACTIVE": 2 if enabled else 1,
    "HAS_LANE_SAFETY": 1,
    "LKAS_ANGLE_CMD": 0,
    "LKAS_ANGLE_MAX_TORQUE": 1000,
    "VALUE104": 0,
    "VALUE231": 0,
    "VALUE239": 0,
    "VALUE247": 0,
    "VALUE255": 0,
  }
  return packer.make_can_msg("LFA", bus, values)


def build_cluster_msg(packer: CANPacker, enabled: bool, bus: int):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": 2 if enabled else 1,
    "NEW_SIGNAL_1": 0,
    "NEW_SIGNAL_2": 0,
    "NEW_SIGNAL_3": 0,
    "NEW_SIGNAL_4": 0,
    "NEW_SIGNAL_5": 0,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", bus, values)


def build_scc_control_msg(packer: CANPacker, accel: float, enabled: bool,
                          counter: int, set_speed_kph: float, bus: int):
  accel = max(-3.5, min(3.5, accel))
  values = {
    "ACC_ObjDist": 0.5,
    "ACC_ObjRelSpd": -170.0,
    "ACC_ObjLatPos": 0.0,
    "SysFailState": 0,
    "MainMode_ACC": 1 if enabled else 0,
    "ACCMode": 1 if enabled else 0,
    "TakeOverReq": 0,
    "InfoDisplay": 0,
    "DriverAlert": 0,
    "ObjDistLevel": 0,
    "DISTANCE_SETTING": 1,
    "VSetDis": max(0, min(255, set_speed_kph)),
    "NSCCOper": 1 if enabled else 0,
    "NSCCOnOff": 2,
    "HUD_LEAD_INFO": 0,
    "DriveMode": 0,
    "aReqValue": accel,
    "aReqRaw": accel,
    "JerkUpperLimit": 3.0,
    "JerkLowerLimit": 1.0,
    "AccelLimitBandUpper": 0.0,
    "AccelLimitBandLower": 0.0,
    "StopReq": 0 if accel >= 0 else 1,
    "CRUSE_INFO_SET_2": 0,
    "TARGET_DISTANCE": 30.0,
    "ZEROS_2": 0,
    "ZEROS": 0,
  }
  return packer.make_can_msg("SCC_CONTROL", bus, values)


def build_cruise_buttons_msg(packer: CANPacker, button: int, counter: int, bus: int):
  values = {
    "COUNTER": counter % 16,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": button,
  }
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)


def parse_args():
  parser = argparse.ArgumentParser(description="단순 LKAS/LFA 송신 스크립트")
  parser.add_argument("--acan-bus", type=int, default=DEFAULT_ACAN, help="LKAS 버스 번호 (default: 0)")
  parser.add_argument("--ecan-bus", type=int, default=DEFAULT_ECAN, help="LFA 버스 번호 (default: 1)")
  parser.add_argument("--torque-scale", type=float, default=400.0, help="조이스틱 → 토크 변환 계수")
  parser.add_argument("--deadzone", type=float, default=0.05, help="조이스틱 데드존")
  parser.add_argument("--max-angle", type=float, default=60.0, help="조이스틱 최대 가상 각도 (참고 출력)")
  parser.add_argument("--send-cluster", action="store_true", help="LFAHDA_CLUSTER 메시지 전송")
  parser.add_argument("--loop-hz", type=float, default=50.0, help="메인 루프 주기 (Hz)")
  parser.add_argument("--disable-fd", action="store_true", help="CAN-FD frame 비활성화 (classic CAN만 전송)")
  parser.add_argument("--long-control", action="store_true", help="SCC_CONTROL 메시지를 이용한 가감속 전송")
  parser.add_argument("--set-speed-kph", type=float, default=30.0, help="가상 크루즈 속도 (HUD용)")
  parser.add_argument("--long-max-accel", type=float, default=2.5, help="최대 가속 명령 (m/s^2)")
  parser.add_argument("--long-max-decel", type=float, default=20.0, help="최대 감속 명령 (m/s^2)")
  parser.add_argument("--accel-axis", type=int, default=5, help="가속 트리거 축 인덱스")
  parser.add_argument("--brake-axis", type=int, default=2, help="브레이크 트리거 축 인덱스")
  parser.add_argument("--cruise-buttons", action="store_true", help="감속/가속 시 크루즈 버튼 메시지 전송")
  parser.add_argument("--vset-rate-up", type=float, default=6.0, help="가속 트리거 시 설정속도 증가율 (kph/s)")
  parser.add_argument("--vset-rate-down", type=float, default=12.0, help="감속 트리거 시 설정속도 감소율 (kph/s)")
  return parser.parse_args()


def main():
  args = parse_args()

  stop_event = threading.Event()

  def handle_sigint(sig, frame):
    stop_event.set()
  signal.signal(signal.SIGINT, handle_sigint)

  joystick = JoystickReader(args.deadzone, args.max_angle, args.accel_axis, args.brake_axis)

  packer = CANPacker("hyundai_canfd_generated")
  panda = Panda()
  panda.set_safety_mode(17, 0)  # SAFETY_ALLOUTPUT
  print("Panda 연결:", panda.get_serial())

  counter = 0
  vset_kph = args.set_speed_kph
  loop_dt = 1.0 / args.loop_hz
  fd_mode = not args.disable_fd
  try:
    while not stop_event.is_set():
      state = joystick.snapshot()
      if args.long_control:
        if state.accel > args.deadzone:
          vset_kph += args.vset_rate_up * state.accel * loop_dt
        if state.brake > args.deadzone:
          vset_kph -= args.vset_rate_down * state.brake * loop_dt
        vset_kph = max(0.0, min(255.0, vset_kph))
      torque_cmd = int(max(-STEER_MAX, min(STEER_MAX, state.steer * args.torque_scale)))
      enabled_lat = state.enabled and abs(torque_cmd) > 0

      accel_cmd = 0.0
      if args.long_control:
        if state.longitudinal >= 0:
          accel_cmd = state.longitudinal * args.long_max_accel
        else:
          accel_cmd = state.longitudinal * args.long_max_decel

      msgs = [
        build_lkas_msg(packer, torque_cmd, enabled_lat, args.acan_bus),
        build_lfa_msg(packer, torque_cmd, enabled_lat, counter, args.ecan_bus),
      ]
      if args.send_cluster and counter % 5 == 0:
        msgs.append(build_cluster_msg(packer, enabled_lat, args.ecan_bus))
      if args.long_control:
        msgs.append(build_scc_control_msg(
          packer,
          accel_cmd,
          state.enabled,
          counter,
          vset_kph,
          args.ecan_bus,
        ))
        if args.cruise_buttons:
          if state.brake > args.deadzone:
            msgs.append(build_cruise_buttons_msg(packer, 2, counter, args.ecan_bus))
          elif state.accel > args.deadzone:
            msgs.append(build_cruise_buttons_msg(packer, 1, counter, args.ecan_bus))

      panda.can_send_many([[addr, dat, bus] for addr, dat, bus in msgs], fd=fd_mode)

      if counter % int(args.loop_hz / 2) == 0:
        print(f"[TX] lat={enabled_lat} torque={torque_cmd} accel={accel_cmd:.2f} vset={vset_kph:.1f} cnt={counter}")

      counter += 1
      time.sleep(loop_dt)
  finally:
    joystick.stop()
    panda.set_safety_mode(0, 0)  # SAFETY_SILENT
    panda.close()
    print("정상 종료.")


if __name__ == "__main__":
  main()
