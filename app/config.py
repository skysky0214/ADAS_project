from dataclasses import dataclass


@dataclass(frozen=True)
class PipelineConfig:
    perception_name: str = "placeholder_detection_adapter"
    tracker_match_distance: float = 1.2
    tracker_reconnect_distance: float = 2.4
    tracker_max_missed: int = 5
    history_size: int = 10
