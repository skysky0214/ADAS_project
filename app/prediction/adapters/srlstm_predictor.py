from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from app.core.domain_types import PredictedTrajectory, TrajectoryPoint, TrackedPedestrian
from app.prediction.base import PredictionModel


class SRLSTMPredictionModel(PredictionModel):
    """Adapter from tracked pedestrians to the bundled SR-LSTM realtime predictor.

    Post-processing features:
    - EMA trajectory smoothing: blends current prediction with the previous one
      to suppress frame-to-frame jitter.
    - Velocity clamping: limits per-step displacement to a physically plausible
      maximum pedestrian speed, preventing extreme outlier jumps.
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
        detections = {
            track.track_id: (float(track.x), float(track.y))
            for track in tracked_objects
            if track.missed == 0
        }
        result = self.predictor.update(detections=detections)
        predictions: dict[int, np.ndarray] = result["predictions"]

        # Purge stale cache entries for tracks no longer predicted
        active_ids = set(predictions.keys())
        stale = [tid for tid in self._prev_predictions if tid not in active_ids]
        for tid in stale:
            del self._prev_predictions[tid]

        trajectories: list[PredictedTrajectory] = []
        dt = self.predictor.dt

        for track_id, pred_xy in predictions.items():
            # pred_xy shape: (pred_length, 2)
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

    def _smooth_trajectory(
        self, track_id: int, raw_pred: np.ndarray, dt: float
    ) -> np.ndarray:
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
        """Limit each step's displacement to max_pedestrian_speed * dt."""
        max_disp = self.max_pedestrian_speed * dt
        result = pred.copy()
        for i in range(1, len(result)):
            delta = result[i] - result[i - 1]
            dist = np.linalg.norm(delta)
            if dist > max_disp:
                result[i] = result[i - 1] + delta * (max_disp / dist)
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

