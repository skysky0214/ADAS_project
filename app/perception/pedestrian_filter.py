from __future__ import annotations

from typing import List

from app.domain_types import DetectedObject, PedestrianDetection


def filter_pedestrians(detections: List[DetectedObject]) -> List[PedestrianDetection]:
    pedestrians = []
    for det in detections:
        if det.label != "Pedestrian":
            continue
        pedestrians.append(
            PedestrianDetection(
                score=det.score,
                x=det.x,
                y=det.y,
            )
        )
    return pedestrians
