from __future__ import annotations

import numpy as np
import pytest

from app.core.domain_types import DetectedObject
from app.perception.adapters.openpcdet_base import _point_spread_stats_in_box_footprint
from app.perception.pedestrian_filter import (
    PedestrianPointSpreadFilter,
    filter_pedestrians,
)


def _det(
    label: str,
    dx: float | None,
    dy: float | None,
    dz: float | None,
    point_max_distance_m: float | None = None,
) -> DetectedObject:
    return DetectedObject(
        label=label,
        score=0.9,
        x=1.0,
        y=2.0,
        z=0.0,
        dx=dx,
        dy=dy,
        dz=dz,
        point_max_distance_m=point_max_distance_m,
    )


def test_filter_pedestrians_keeps_only_pedestrian_label_by_default() -> None:
    detections = [
        _det("Pedestrian", None, None, None),
        _det("Cyclist", 0.8, 0.6, 1.7),
    ]

    pedestrians = filter_pedestrians(detections)

    assert len(pedestrians) == 1
    assert pedestrians[0].x == 1.0


def test_filter_pedestrians_applies_point_max_distance_range_when_enabled() -> None:
    detections = [
        _det("Pedestrian", 0.2, 0.2, 1.0, point_max_distance_m=0.25),
        _det("Pedestrian", 0.5, 0.6, 1.8, point_max_distance_m=1.2),
        _det("Pedestrian", 1.0, 1.0, 2.0, point_max_distance_m=2.1),
        _det("Pedestrian", None, 0.6, 1.8),
    ]
    point_spread_filter = PedestrianPointSpreadFilter(
        min_point_max_distance_m=1.0,
        max_point_max_distance_m=1.8,
    )

    pedestrians = filter_pedestrians(detections, point_spread_filter=point_spread_filter)

    assert len(pedestrians) == 1
    assert pedestrians[0].dx == 0.5
    assert pedestrians[0].dy == 0.6
    assert pedestrians[0].dz == 1.8
    assert pedestrians[0].point_max_distance_m == 1.2


def test_point_spread_filter_rejects_invalid_thresholds() -> None:
    with pytest.raises(ValueError):
        PedestrianPointSpreadFilter(min_point_max_distance_m=-0.1)

    with pytest.raises(ValueError):
        PedestrianPointSpreadFilter(
            min_point_max_distance_m=1.0,
            max_point_max_distance_m=0.5,
        )


def test_point_spread_stats_in_box_footprint_uses_points_inside_rotated_footprint() -> None:
    points = np.array(
        [
            [0.0, 0.0, 0.0, 0.0],
            [0.6, 0.0, 0.0, 0.0],
            [0.0, 0.8, 0.0, 0.0],
            [2.0, 0.0, 5.0, 0.0],
        ],
        dtype=np.float32,
    )
    box = np.array([0.0, 0.0, 0.0, 2.0, 2.0, 1.7, 0.0], dtype=np.float32)

    point_max_distance_m, point_count = _point_spread_stats_in_box_footprint(points, box)

    assert point_max_distance_m == pytest.approx(1.0)
    assert point_count == 3
