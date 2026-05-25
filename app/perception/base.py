from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from app.core.domain_types import DetectedObject, FrameInput


class PerceptionModel(ABC):
    """Adapter boundary for any perception model."""

    @abstractmethod
    def infer(self, frame: FrameInput) -> List[DetectedObject]:
        """Convert raw sensor input into current-frame detections."""
