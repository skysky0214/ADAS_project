from __future__ import annotations

from dataclasses import asdict
from pprint import pformat

from app.config import PipelineConfig
from app.domain_types import FrameInput, TrackingFrameResult
from app.perception.pedestrian_filter import filter_pedestrians
from app.perception.placeholder import PlaceholderPerceptionModel
from app.tracking.pedestrian_tracker import PedestrianTracker


class RealTimePedestrianTrackingPipeline:
    """
    Real-time frame pipeline up to pedestrian tracking.

    frame -> detection -> pedestrian filtering -> tracking -> id-wise xy history
    """

    def __init__(self, config: PipelineConfig):
        self.config = config
        self.detector = PlaceholderPerceptionModel()
        self.tracker = PedestrianTracker(
            match_distance=config.tracker_match_distance,
            reconnect_distance=config.tracker_reconnect_distance,
            max_missed=config.tracker_max_missed,
            history_size=config.history_size,
        )

    def step(self, frame: FrameInput) -> TrackingFrameResult:
        detections = self.detector.infer(frame)
        pedestrian_detections = filter_pedestrians(detections)
        tracked_objects = self.tracker.update(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
        )
        return TrackingFrameResult(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
            tracks=tracked_objects,
        )

    @staticmethod
    def debug_dump(result: TrackingFrameResult) -> str:
        return pformat(asdict(result), sort_dicts=False)
