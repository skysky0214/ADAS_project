from __future__ import annotations

from typing import List

from app.domain_types import PredictedTrajectory, TrajectoryPoint, TrackedObject
from app.prediction.base import PredictionModel


class PlaceholderPredictionModel(PredictionModel):
    """
    Temporary adapter used before the real trajectory model is connected.

    For now it simply extrapolates linearly from the latest tracked velocity.
    Replace this with a learned prediction model later.
    """

    def __init__(self, horizon_sec: float, step_sec: float):
        self.horizon_sec = horizon_sec
        self.step_sec = step_sec

    def predict(self, tracked_objects: List[TrackedObject]) -> List[PredictedTrajectory]:
        trajectories: List[PredictedTrajectory] = []

        for obj in tracked_objects:
            points = []
            steps = int(self.horizon_sec / self.step_sec)
            for idx in range(1, steps + 1):
                t_sec = idx * self.step_sec
                points.append(
                    TrajectoryPoint(
                        t_sec=t_sec,
                        x=obj.x + (obj.vx * idx),
                        y=obj.y + (obj.vy * idx),
                    )
                )
            trajectories.append(
                PredictedTrajectory(
                    track_id=obj.track_id,
                    label=obj.label,
                    confidence=min(0.95, max(0.3, obj.score)),
                    points=points,
                )
            )

        return trajectories
