from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from app.core.domain_types import PredictedTrajectory, TrajectoryPoint, TrackedPedestrian
from app.prediction.base import PredictionModel


class SRLSTMPredictionModel(PredictionModel):
    """Adapter from tracked pedestrians to the bundled SR-LSTM realtime predictor."""

    def __init__(
        self,
        checkpoint: Path,
        sensor_fps: float = 2.5,
        model_dir: Path | None = None,
    ):
        self.checkpoint = checkpoint.resolve()
        self.sensor_fps = sensor_fps
        self.model_dir = (model_dir or Path(__file__).resolve().parents[1] / "srlstm").resolve()
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

        result = self.predictor.update(detections=detections)
        predictions: dict[int, np.ndarray] = result["predictions"]

        trajectories: list[PredictedTrajectory] = []
        for track_id, pred_xy in predictions.items():
            points = [
                TrajectoryPoint(
                    t_sec=(idx + 1) * self.predictor.dt,
                    x=float(x),
                    y=float(y),
                )
                for idx, (x, y) in enumerate(pred_xy)
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
