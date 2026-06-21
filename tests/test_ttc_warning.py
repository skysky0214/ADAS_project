from __future__ import annotations

import math

import pytest

from app.control.ttc_warning import TTCWarningAdapter, TTCWarningConfig
from app.core.domain_types import TrackedPedestrian


def _adapter(speed_mps: float = 10.0, safety_radius_m: float = 1.0) -> TTCWarningAdapter:
    return TTCWarningAdapter(
        TTCWarningConfig(
            ego_speed_mps=speed_mps,
            prediction_dt_sec=0.4,
            safety_radius_m=safety_radius_m,
            vehicle_front_m=2.4,
            vehicle_rear_m=2.1,
            vehicle_side_m=1.0,
        )
    )


def test_dynamic_ttc_uses_future_ego_path_for_stationary_pedestrian() -> None:
    adapter = _adapter(speed_mps=10.0, safety_radius_m=0.0)
    track = TrackedPedestrian(track_id=1, x=10.0, y=0.0, dx=0.6, dy=0.6)
    warnings = adapter.evaluate(
        frame_id=7,
        timestamp_sec=100.0,
        tracked_objects=[track],
        predicted_trajectories=[],
    )

    assert len(warnings) == 1
    assert warnings[0].min_ttc_sec == pytest.approx(0.73)
    assert warnings[0].collision_time_sec == pytest.approx(100.73)


def test_dynamic_ttc_applies_safety_radius_to_lateral_clearance() -> None:
    track = TrackedPedestrian(track_id=1, x=10.0, y=2.0, dx=0.6, dy=0.6)

    no_margin = _adapter(speed_mps=10.0, safety_radius_m=0.0).evaluate(
        frame_id=1,
        timestamp_sec=0.0,
        tracked_objects=[track],
        predicted_trajectories=[],
    )[0]
    with_margin = _adapter(speed_mps=10.0, safety_radius_m=1.0).evaluate(
        frame_id=1,
        timestamp_sec=0.0,
        tracked_objects=[track],
        predicted_trajectories=[],
    )[0]

    assert math.isinf(no_margin.min_ttc_sec)
    assert with_margin.min_ttc_sec == pytest.approx(0.7538461538461539)


def test_ttc_ignores_tracks_not_detected_in_current_frame() -> None:
    adapter = _adapter(speed_mps=10.0, safety_radius_m=1.0)
    stale_track = TrackedPedestrian(
        track_id=1,
        x=10.0,
        y=0.0,
        dx=0.6,
        dy=0.6,
        missed=1,
    )

    warnings = adapter.evaluate(
        frame_id=1,
        timestamp_sec=0.0,
        tracked_objects=[stale_track],
        predicted_trajectories=[],
    )

    assert warnings == []


def test_ttc_levels_do_not_scale_with_speed_or_driver_brake() -> None:
    adapter = TTCWarningAdapter(
        TTCWarningConfig(
            ego_speed_mps=0.0,
            driver_brake_pressed=True,
            driver_accelerator_pressed=False,
        )
    )

    assert adapter.classify_warning(1.6)["level"] == 1
    assert adapter.classify_warning(1.0)["level"] == 2
    assert adapter.classify_warning(0.75)["level"] == 3
