from __future__ import annotations

from math import hypot

from app.domain_types import PedestrianDetection


def euclidean_distance(
    det: PedestrianDetection,
    predicted_x: float,
    predicted_y: float,
) -> float:
    return hypot(det.x - predicted_x, det.y - predicted_y)
