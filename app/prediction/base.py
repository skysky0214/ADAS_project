from __future__ import annotations

from abc import ABC, abstractmethod

from app.core.domain_types import PredictedTrajectory, TrackedPedestrian


class PredictionModel(ABC):
    """Adapter boundary for any trajectory prediction model."""

    @abstractmethod
    def predict(self, tracked_objects: list[TrackedPedestrian]) -> list[PredictedTrajectory]:
        """Convert tracked pedestrians into future trajectories."""
