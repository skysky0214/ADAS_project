from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass(frozen=True)
class FrameInput:
    frame_id: int
    timestamp_sec: float
    sensor_source: str
    payload: dict


@dataclass(frozen=True)
class DetectedObject:
    label: str
    score: float
    x: float
    y: float
    z: float = 0.0
    dx: float | None = None
    dy: float | None = None
    dz: float | None = None
    heading: float | None = None
    point_max_distance_m: float | None = None
    point_count: int | None = None


@dataclass(frozen=True)
class PedestrianDetection:
    score: float
    x: float
    y: float
    z: float = 0.0
    dx: float | None = None
    dy: float | None = None
    dz: float | None = None
    heading: float | None = None
    point_max_distance_m: float | None = None
    point_count: int | None = None


@dataclass(frozen=True)
class HistoryPoint:
    frame_id: int
    timestamp_sec: float
    x: float
    y: float


@dataclass
class TrackedPedestrian:
    track_id: int
    x: float
    y: float
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    missed: int = 0
    score: float = 0.0
    dx: float | None = None
    dy: float | None = None
    dz: float | None = None
    heading: float | None = None
    point_max_distance_m: float | None = None
    point_count: int | None = None
    history: List[HistoryPoint] = field(default_factory=list)


@dataclass(frozen=True)
class TrackingFrameResult:
    frame_id: int
    timestamp_sec: float
    detections: List[PedestrianDetection]
    tracks: List[TrackedPedestrian]


@dataclass(frozen=True)
class PredictionInputSequence:
    track_id: int
    observed_xy: List[tuple[float, float]]
    current_xy: tuple[float, float]
    velocity_xy: tuple[float, float]
    history_len: int


@dataclass(frozen=True)
class PredictionInputBatch:
    frame_id: int
    timestamp_sec: float
    sequences: List[PredictionInputSequence]


@dataclass(frozen=True)
class TrajectoryPoint:
    t_sec: float
    x: float
    y: float


@dataclass(frozen=True)
class PredictedTrajectory:
    track_id: int
    points: List[TrajectoryPoint]
    confidence: float = 1.0
    model_name: str = "unknown"


@dataclass(frozen=True)
class PredictionFrameResult:
    frame_id: int
    timestamp_sec: float
    trajectories: List[PredictedTrajectory]


@dataclass(frozen=True)
class PlannerObjectView:
    track_id: int
    label: str
    current_position: tuple[float, float, float]
    current_velocity: tuple[float, float]
    predicted_path: List[tuple[float, float]]
    missed: int = 0


@dataclass(frozen=True)
class PlannerSceneSnapshot:
    frame_id: int
    timestamp_sec: float
    objects: List[PlannerObjectView]
