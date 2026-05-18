from __future__ import annotations

import json
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class FailSafeConfig:
    lidar_timeout_sec: float = 1.0


@dataclass(frozen=True)
class SystemStatus:
    status: str
    system_alive: bool
    lidar_alive: bool
    message: str
    last_lidar_age_sec: float | None


class FailSafeMonitor:
    def __init__(self, config: FailSafeConfig | None = None):
        self.config = config or FailSafeConfig()
        self.last_lidar_time_sec: float | None = None

    def mark_lidar_received(self, now_sec: float) -> None:
        self.last_lidar_time_sec = now_sec

    def build_status(self, now_sec: float) -> SystemStatus:
        if self.last_lidar_time_sec is None:
            return SystemStatus(
                status="SENSOR_TIMEOUT",
                system_alive=True,
                lidar_alive=False,
                message="LiDAR data has not been received.",
                last_lidar_age_sec=None,
            )

        last_lidar_age_sec = now_sec - self.last_lidar_time_sec
        lidar_alive = last_lidar_age_sec <= self.config.lidar_timeout_sec

        if lidar_alive:
            return SystemStatus(
                status="OK",
                system_alive=True,
                lidar_alive=True,
                message="ADAS system operating.",
                last_lidar_age_sec=round(last_lidar_age_sec, 3),
            )

        return SystemStatus(
            status="SENSOR_TIMEOUT",
            system_alive=True,
            lidar_alive=False,
            message="LiDAR data timeout. Driver takeover required.",
            last_lidar_age_sec=round(last_lidar_age_sec, 3),
        )


def status_to_json(status: SystemStatus) -> str:
    return json.dumps(asdict(status), ensure_ascii=False)