from __future__ import annotations

from typing import Dict, List

from app.domain_types import (
    PlannerObjectView,
    PlannerSceneSnapshot,
    PredictedTrajectory,
    TrackedObject,
)


def build_planner_snapshot(
    frame_id: int,
    timestamp_sec: float,
    tracked_objects: List[TrackedObject],
    predicted_trajectories: List[PredictedTrajectory],
) -> PlannerSceneSnapshot:
    pred_map: Dict[int, PredictedTrajectory] = {
        item.track_id: item for item in predicted_trajectories
    }

    planner_objects = []
    for obj in tracked_objects:
        trajectory = pred_map.get(obj.track_id)
        predicted_path = []
        if trajectory is not None:
            predicted_path = [(point.x, point.y) for point in trajectory.points]

        planner_objects.append(
            PlannerObjectView(
                track_id=obj.track_id,
                label=obj.label,
                current_position=(obj.x, obj.y, obj.z),
                current_velocity=(obj.vx, obj.vy),
                predicted_path=predicted_path,
            )
        )

    return PlannerSceneSnapshot(
        frame_id=frame_id,
        timestamp_sec=timestamp_sec,
        objects=planner_objects,
    )
