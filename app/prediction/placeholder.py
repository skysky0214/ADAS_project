from __future__ import annotations

from app.core.domain_types import PredictedTrajectory, TrajectoryPoint, TrackedPedestrian
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

    def predict(
        self,
        tracked_objects: list[TrackedPedestrian],
        timestamp_sec: float | None = None,
        ego_motion_delta: tuple[float, float, float] | None = None,
    ) -> list[PredictedTrajectory]:
        trajectories: list[PredictedTrajectory] = []

        for obj in tracked_objects:
            points = []
            steps = int(self.horizon_sec / self.step_sec)
            for idx in range(1, steps + 1):
                t_sec = idx * self.step_sec
                points.append(
                    TrajectoryPoint(
                        t_sec=t_sec,
                        x=obj.x + (obj.vx * t_sec),
                        y=obj.y + (obj.vy * t_sec),
                    )
                )
            trajectories.append(
                PredictedTrajectory(
                    track_id=obj.track_id,
                    points=points,
                    confidence=0.5,
                    model_name="linear_placeholder",
                )
            )

        return trajectories
