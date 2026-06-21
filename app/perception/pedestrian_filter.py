from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from app.core.domain_types import DetectedObject, PedestrianDetection


@dataclass(frozen=True)
class PedestrianPointSpreadFilter:
    min_point_max_distance_m: float | None = None
    max_point_max_distance_m: float | None = None

    def __post_init__(self) -> None:
        _validate_non_negative_threshold("min_point_max_distance_m", self.min_point_max_distance_m)
        _validate_non_negative_threshold("max_point_max_distance_m", self.max_point_max_distance_m)
        if (
            self.min_point_max_distance_m is not None
            and self.max_point_max_distance_m is not None
            and self.min_point_max_distance_m > self.max_point_max_distance_m
        ):
            raise ValueError(
                "min_point_max_distance_m must be less than or equal to max_point_max_distance_m"
            )

    @property
    def enabled(self) -> bool:
        return self.min_point_max_distance_m is not None or self.max_point_max_distance_m is not None

    def keeps(self, detection: DetectedObject) -> bool:
        if not self.enabled:
            return True

        point_max_distance_m = detection.point_max_distance_m
        if point_max_distance_m is None or not math.isfinite(point_max_distance_m):
            return False
        if (
            self.min_point_max_distance_m is not None
            and point_max_distance_m < self.min_point_max_distance_m
        ):
            return False
        if (
            self.max_point_max_distance_m is not None
            and point_max_distance_m > self.max_point_max_distance_m
        ):
            return False
        return True


def _validate_non_negative_threshold(name: str, value: float | None) -> None:
    if value is None:
        return
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")


def filter_pedestrians(
    detections: List[DetectedObject],
    point_spread_filter: PedestrianPointSpreadFilter | None = None,
) -> List[PedestrianDetection]:
    point_spread_filter = point_spread_filter or PedestrianPointSpreadFilter()
    pedestrians = []
    for det in detections:
        if det.label != "Pedestrian":
            continue
        if not point_spread_filter.keeps(det):
            continue
        pedestrians.append(
            PedestrianDetection(
                score=det.score,
                x=det.x,
                y=det.y,
                z=det.z,
                dx=det.dx,
                dy=det.dy,
                dz=det.dz,
                heading=det.heading,
                point_max_distance_m=det.point_max_distance_m,
                point_count=det.point_count,
            )
        )
    return pedestrians
