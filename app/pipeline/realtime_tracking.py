from __future__ import annotations

from dataclasses import asdict
from pprint import pformat

from app.core.config import PipelineConfig
from app.core.domain_types import FrameInput, TrackingFrameResult
from app.perception.base import PerceptionModel
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
        self.detector = self._build_detector(config)
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

    @staticmethod
    def _build_detector(config: PipelineConfig) -> PerceptionModel:
        if config.perception_name == "placeholder_detection_adapter":
            return PlaceholderPerceptionModel()
        if config.perception_name == "openpcdet_dsvt":
            from app.perception.adapters.openpcdet_dsvt import OpenPCDetDSVTPerceptionModel

            return OpenPCDetDSVTPerceptionModel(
                openpcdet_root=config.openpcdet_root,
                cfg_file=config.openpcdet_cfg_file,
                checkpoint=config.openpcdet_checkpoint,
                score_threshold=config.perception_score_threshold,
                device=config.perception_device,
            )
        if config.perception_name == "openpcdet_pointpillar":
            from app.perception.adapters.openpcdet_pointpillar import OpenPCDetPointPillarPerceptionModel

            return OpenPCDetPointPillarPerceptionModel(
                openpcdet_root=config.openpcdet_root,
                cfg_file=config.pointpillar_cfg_file,
                checkpoint=config.pointpillar_checkpoint,
                score_threshold=config.perception_score_threshold,
                device=config.perception_device,
            )
        if config.perception_name == "clustering":
            from app.perception.adapters.clustering_detector import ClusteringPerceptionModel

            return ClusteringPerceptionModel(
                roi_x_min=config.roi_x_min,
                roi_x_max=config.roi_x_max,
                roi_y_min=config.roi_y_min,
                roi_y_max=config.roi_y_max,
                roi_z_min=config.roi_z_min,
                roi_z_max=config.roi_z_max,
            )
        raise ValueError(f"Unknown perception adapter: {config.perception_name}")
