from __future__ import annotations

from typing import List

from app.core.domain_types import DetectedObject, FrameInput
from app.perception.base import PerceptionModel


class PlaceholderPerceptionModel(PerceptionModel):
    """
    Temporary adapter used before the real perception model is connected.

    Replace this class with a PointPillars, BEVFusion, or custom detector
    implementation later without changing tracking/prediction/planner code.
    """

    def infer(self, frame: FrameInput) -> List[DetectedObject]:
        detections = frame.payload.get("detections")
        if detections is None:
            raise NotImplementedError(
                "Real perception model is not connected yet. "
                "Pass mock detections in frame.payload['detections'] "
                "or replace this adapter with a real model."
            )

        return [
            DetectedObject(
                label=item["label"],
                score=float(item["score"]),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item.get("z", 0.0)),
            )
            for item in detections
        ]
