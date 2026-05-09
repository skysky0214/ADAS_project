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


@dataclass(frozen=True)
class PedestrianDetection:
    score: float
    x: float
    y: float


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
    vx: float = 0.0
    vy: float = 0.0
    missed: int = 0
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
