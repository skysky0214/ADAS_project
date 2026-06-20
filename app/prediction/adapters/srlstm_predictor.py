from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from app.core.domain_types import PredictedTrajectory, TrajectoryPoint, TrackedPedestrian
from app.prediction.base import PredictionModel


class SRLSTMPredictionModel(PredictionModel):
    """Adapter from tracked pedestrians to the bundled SR-LSTM realtime predictor.

    Post-processing features:
    - EMA trajectory smoothing to suppress frame-to-frame jitter.
    - Velocity clamping to prevent implausible prediction jumps.
    """

    def __init__(
        self,
        checkpoint: Path,
        sensor_fps: float = 2.5,
        model_dir: Path | None = None,
        smooth_alpha: float = 0.4,
        max_pedestrian_speed: float = 3.0,
    ):
        self.checkpoint = checkpoint.resolve()
        self.sensor_fps = sensor_fps
        self.model_dir = (model_dir or Path(__file__).resolve().parents[1] / "srlstm").resolve()
        self.smooth_alpha = smooth_alpha
        self.max_pedestrian_speed = max_pedestrian_speed
        self._prev_predictions: dict[int, np.ndarray] = {}
        self._load_model()

    def predict(self, tracked_objects: list[TrackedPedestrian]) -> list[PredictedTrajectory]:
        detections = {}
        active_track_ids = set()
        for track in tracked_objects:
            if track.missed != 0:
                continue
            active_track_ids.add(track.track_id)
            detections[track.track_id] = (float(track.x), float(track.y))

            # Keep SR-LSTM's observation buffer aligned to the tracker history.
            # The tracker history is already ego-motion compensated into the
            # current LiDAR frame; appending raw per-frame detections here would
            # reintroduce ego-relative drift for stationary objects.
            history_xy = [(float(point.x), float(point.y)) for point in track.history]
            self.predictor.obs_buffer[track.track_id] = history_xy[:-1]

        for stale_track_id in list(self.predictor.obs_buffer):
            if stale_track_id not in active_track_ids:
                del self.predictor.obs_buffer[stale_track_id]
                self.predictor.last_predictions.pop(stale_track_id, None)
                self.predictor.last_ttc.pop(stale_track_id, None)
                self._prev_predictions.pop(stale_track_id, None)

        result = self.predictor.update(detections=detections)
        predictions: dict[int, np.ndarray] = result["predictions"]

        active_prediction_ids = set(predictions)
        for stale_track_id in list(self._prev_predictions):
            if stale_track_id not in active_prediction_ids:
                del self._prev_predictions[stale_track_id]

        trajectories: list[PredictedTrajectory] = []
        dt = self.predictor.dt
        for track_id, pred_xy in predictions.items():
            smoothed = self._smooth_trajectory(track_id, pred_xy, dt)
            points = [
                TrajectoryPoint(
                    t_sec=(idx + 1) * dt,
                    x=float(smoothed[idx, 0]),
                    y=float(smoothed[idx, 1]),
                )
                for idx in range(len(smoothed))
            ]
            trajectories.append(
                PredictedTrajectory(
                    track_id=int(track_id),
                    points=points,
                    confidence=1.0,
                    model_name="srlstm",
                )
            )
        return trajectories

    def _smooth_trajectory(self, track_id: int, raw_pred: np.ndarray, dt: float) -> np.ndarray:
        """Apply velocity clamping then EMA smoothing to a raw prediction."""
        clamped = self._clamp_velocity(raw_pred, dt)

        prev = self._prev_predictions.get(track_id)
        if prev is not None and prev.shape == clamped.shape:
            alpha = self.smooth_alpha
            smoothed = alpha * clamped + (1.0 - alpha) * prev
        else:
            smoothed = clamped

        self._prev_predictions[track_id] = smoothed.copy()
        return smoothed

    def _clamp_velocity(self, pred: np.ndarray, dt: float) -> np.ndarray:
        """Limit each prediction step to max_pedestrian_speed * dt."""
        max_disp = self.max_pedestrian_speed * dt
        result = pred.copy()
        for idx in range(1, len(result)):
            delta = result[idx] - result[idx - 1]
            dist = np.linalg.norm(delta)
            if dist > max_disp:
                result[idx] = result[idx - 1] + delta * (max_disp / dist)
        return result

    def _load_model(self) -> None:
        if not self.checkpoint.exists():
            raise FileNotFoundError(f"SR-LSTM checkpoint not found: {self.checkpoint}")
        if not self.model_dir.exists():
            raise FileNotFoundError(f"SR-LSTM model dir not found: {self.model_dir}")
        if str(self.model_dir) not in sys.path:
            sys.path.insert(0, str(self.model_dir))

        from realtime_predictor import RealtimePredictor, load_srlstm_model

        model, args = load_srlstm_model(str(self.checkpoint))
        self.predictor = RealtimePredictor(model, args, sensor_fps=self.sensor_fps)
