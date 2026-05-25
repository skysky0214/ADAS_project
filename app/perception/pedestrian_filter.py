from __future__ import annotations

from typing import List

from app.core.domain_types import DetectedObject, PedestrianDetection


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
                z=det.z,
                dx=det.dx,
                dy=det.dy,
                dz=det.dz,
                heading=det.heading,
            )
        )
    return pedestrians
