from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from app.domain_types import PredictedTrajectory, TrackedObject


class PredictionModel(ABC):
    """Adapter boundary for any trajectory prediction model."""

    @abstractmethod
    def predict(self, tracked_objects: List[TrackedObject]) -> List[PredictedTrajectory]:
        """Convert tracked objects into future trajectories."""
