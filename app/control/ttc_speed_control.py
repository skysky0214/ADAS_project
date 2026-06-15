from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TTCSpeedControlConfig:
    """Convert TTC risk into a target speed and brake percentage.

    The controller assumes the previously computed TTC is valid for the current
    ego speed. To increase TTC from current_ttc to desired_ttc, it scales ego
    speed by current_ttc / desired_ttc, then converts the speed reduction into a
    brake percentage using a bounded deceleration model.
    """

    ego_speed_mps: float = 10.0
    desired_ttc_sec: float = 2.5
    warning_ttc_sec: float = 1.7
    critical_ttc_sec: float = 0.8
    response_time_sec: float = 1.0
    max_service_decel_mps2: float = 5.0
    max_emergency_decel_mps2: float = 8.0
    min_target_speed_mps: float = 0.0


@dataclass(frozen=True)
class TTCSpeedControlCommand:
    track_id: int
    input_ttc_sec: float
    desired_ttc_sec: float
    current_speed_mps: float
    target_speed_mps: float
    target_speed_kmh: float
    brake_percent: float
    target_decel_mps2: float
    action: str
    reason: str


class TTCSpeedController:
    def __init__(self, config: TTCSpeedControlConfig | None = None):
        self.config = config or TTCSpeedControlConfig()

    def command_for_object(
        self,
        track_id: int,
        ttc_sec: float,
        current_speed_mps: float | None = None,
    ) -> TTCSpeedControlCommand:
        config = self.config
        current_speed = max(current_speed_mps if current_speed_mps is not None else config.ego_speed_mps, 0.0)

        if math.isinf(ttc_sec) or ttc_sec >= config.desired_ttc_sec:
            return self._command(
                track_id=track_id,
                ttc_sec=ttc_sec,
                current_speed=current_speed,
                target_speed=current_speed,
                brake_percent=0.0,
                decel=0.0,
                action="keep_speed",
                reason="TTC is already above the desired margin.",
            )

        if ttc_sec <= 0.0 or current_speed <= 0.0:
            target_speed = config.min_target_speed_mps
            decel = self._required_decel(current_speed, target_speed, config.response_time_sec)
            return self._command(
                track_id=track_id,
                ttc_sec=ttc_sec,
                current_speed=current_speed,
                target_speed=target_speed,
                brake_percent=self._brake_percent(decel, emergency=True),
                decel=decel,
                action="emergency_brake",
                reason="TTC is zero/negative or ego speed cannot safely preserve TTC.",
            )

        target_speed = current_speed * (ttc_sec / config.desired_ttc_sec)
        target_speed = max(config.min_target_speed_mps, min(target_speed, current_speed))
        decel = self._required_decel(current_speed, target_speed, config.response_time_sec)

        if ttc_sec <= config.critical_ttc_sec:
            action = "emergency_brake"
            emergency = True
        elif ttc_sec <= config.warning_ttc_sec:
            action = "service_brake"
            emergency = False
        else:
            action = "soft_slowdown"
            emergency = False

        return self._command(
            track_id=track_id,
            ttc_sec=ttc_sec,
            current_speed=current_speed,
            target_speed=target_speed,
            brake_percent=self._brake_percent(decel, emergency=emergency),
            decel=decel,
            action=action,
            reason=(
                f"Reduce speed by TTC ratio {ttc_sec:.2f}/{config.desired_ttc_sec:.2f} "
                "to increase the time margin."
            ),
        )

    def commands_from_warnings(
        self,
        warnings: list[dict[str, Any]],
        current_speed_mps: float | None = None,
    ) -> list[TTCSpeedControlCommand]:
        commands = []
        for warning in warnings:
            ttc_sec = _parse_ttc(warning.get("min_ttc_sec"))
            if ttc_sec is None:
                continue
            commands.append(
                self.command_for_object(
                    track_id=int(warning.get("track_id", -1)),
                    ttc_sec=ttc_sec,
                    current_speed_mps=current_speed_mps,
                )
            )
        commands.sort(key=lambda item: item.input_ttc_sec)
        return commands

    def most_urgent_command(
        self,
        warnings: list[dict[str, Any]],
        current_speed_mps: float | None = None,
    ) -> TTCSpeedControlCommand | None:
        commands = self.commands_from_warnings(warnings, current_speed_mps)
        return commands[0] if commands else None

    def _required_decel(self, current_speed: float, target_speed: float, response_time: float) -> float:
        dt = max(response_time, 1e-3)
        return max((current_speed - target_speed) / dt, 0.0)

    def _brake_percent(self, decel_mps2: float, emergency: bool) -> float:
        max_decel = self.config.max_emergency_decel_mps2 if emergency else self.config.max_service_decel_mps2
        if max_decel <= 0.0:
            return 0.0
        return round(min(decel_mps2 / max_decel, 1.0) * 100.0, 1)

    def _command(
        self,
        track_id: int,
        ttc_sec: float,
        current_speed: float,
        target_speed: float,
        brake_percent: float,
        decel: float,
        action: str,
        reason: str,
    ) -> TTCSpeedControlCommand:
        return TTCSpeedControlCommand(
            track_id=track_id,
            input_ttc_sec=ttc_sec,
            desired_ttc_sec=self.config.desired_ttc_sec,
            current_speed_mps=round(current_speed, 3),
            target_speed_mps=round(target_speed, 3),
            target_speed_kmh=round(target_speed * 3.6, 3),
            brake_percent=brake_percent,
            target_decel_mps2=round(decel, 3),
            action=action,
            reason=reason,
        )


def load_ttc_warnings(json_path: Path) -> list[dict[str, Any]]:
    data = json.loads(json_path.read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("warnings", [])
    if not isinstance(data, list):
        raise ValueError("TTC warning input must be a list or an object with a 'warnings' list.")
    return [item for item in data if isinstance(item, dict)]


def command_rows(commands: list[TTCSpeedControlCommand]) -> list[dict[str, Any]]:
    rows = []
    for command in commands:
        row = asdict(command)
        if math.isinf(row["input_ttc_sec"]):
            row["input_ttc_sec"] = "inf"
        rows.append(row)
    return rows


def write_commands_json(commands: list[TTCSpeedControlCommand], json_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(command_rows(commands), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _parse_ttc(value: Any) -> float | None:
    if value in (None, ""):
        return None
    if isinstance(value, str) and value.lower() == "inf":
        return float("inf")
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read TTC warnings and print target speed/brake commands."
    )
    parser.add_argument("warnings_json", type=Path, help="Path to ttc_warnings.json")
    parser.add_argument("--ego-speed", type=float, default=10.0, help="Current ego speed in m/s")
    parser.add_argument("--desired-ttc", type=float, default=2.5, help="Target TTC margin in seconds")
    parser.add_argument("--warning-ttc", type=float, default=1.7)
    parser.add_argument("--critical-ttc", type=float, default=0.8)
    parser.add_argument("--response-time", type=float, default=1.0)
    parser.add_argument("--output-json", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    controller = TTCSpeedController(
        TTCSpeedControlConfig(
            ego_speed_mps=args.ego_speed,
            desired_ttc_sec=args.desired_ttc,
            warning_ttc_sec=args.warning_ttc,
            critical_ttc_sec=args.critical_ttc,
            response_time_sec=args.response_time,
        )
    )
    warnings = load_ttc_warnings(args.warnings_json)
    commands = controller.commands_from_warnings(warnings)

    for command in commands:
        ttc_text = "inf" if math.isinf(command.input_ttc_sec) else f"{command.input_ttc_sec:.2f}s"
        print(
            f"track={command.track_id} TTC={ttc_text} action={command.action} "
            f"target_speed={command.target_speed_mps:.2f}m/s({command.target_speed_kmh:.1f}km/h) "
            f"brake={command.brake_percent:.1f}% decel={command.target_decel_mps2:.2f}m/s^2"
        )

    if args.output_json is not None:
        write_commands_json(commands, args.output_json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
