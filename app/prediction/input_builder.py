from __future__ import annotations

from typing import List

from app.domain_types import (
    PredictionInputBatch,
    PredictionInputSequence,
    TrackingFrameResult,
)


def build_prediction_input(result: TrackingFrameResult) -> PredictionInputBatch:
    sequences: List[PredictionInputSequence] = []

    for track in result.tracks:
        observed_xy = [(point.x, point.y) for point in track.history]
        sequences.append(
            PredictionInputSequence(
                track_id=track.track_id,
                observed_xy=observed_xy,
                current_xy=(track.x, track.y),
                velocity_xy=(track.vx, track.vy),
                history_len=len(track.history),
            )
        )

    return PredictionInputBatch(
        frame_id=result.frame_id,
        timestamp_sec=result.timestamp_sec,
        sequences=sequences,
    )
