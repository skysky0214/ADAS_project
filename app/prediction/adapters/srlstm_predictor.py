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
        self._prev_track_xy: dict[int, tuple[float, float]] = {}
        self._sample_buffer: dict[int, list[tuple[float, float, float]]] = {}
        self._load_model()

    def predict(
        self,
        tracked_objects: list[TrackedPedestrian],
        timestamp_sec: float | None = None,
        ego_motion_delta: tuple[float, float, float] | None = None,
    ) -> list[PredictedTrajectory]:
        active_tracks = [track for track in tracked_objects if track.missed == 0]
        current_xy_by_id = {
            track.track_id: (float(track.x), float(track.y))
            for track in active_tracks
        }

        active_ids = set(current_xy_by_id)
        self._purge_stale_state(active_ids)
        self._apply_ego_motion_to_cached_state(ego_motion_delta)

        if timestamp_sec is None:
            self._sync_predictor_buffer_from_tracks(active_tracks)
            detections = {
                track.track_id: (float(track.x), float(track.y))
                for track in active_tracks
            }
            result = self.predictor.update(detections=detections)
        else:
            has_new_sample = self._sync_sampled_predictor_buffer(active_tracks, timestamp_sec)
            if not has_new_sample:
                return self._cached_trajectories(active_tracks)
            result = self.predictor.predict_from_buffer(track_ids=list(active_ids))
        predictions: dict[int, np.ndarray] = result["predictions"]

        return self._predictions_to_trajectories(predictions, current_xy_by_id)

    def _predictions_to_trajectories(
        self,
        predictions: dict[int, np.ndarray],
        current_xy_by_id: dict[int, tuple[float, float]],
    ) -> list[PredictedTrajectory]:
        if not predictions:
            return []

        trajectories: list[PredictedTrajectory] = []
        dt = self.predictor.dt

        for track_id, pred_xy in predictions.items():
            # pred_xy shape: (pred_length, 2)
            smoothed = self._smooth_trajectory(track_id, pred_xy, dt, current_xy_by_id.get(track_id))
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

    def _cached_trajectories(self, tracks: list[TrackedPedestrian]) -> list[PredictedTrajectory]:
        trajectories: list[PredictedTrajectory] = []
        dt = self.predictor.dt

        for track in tracks:
            track_id = track.track_id
            previous = self._prev_predictions.get(track_id)
            previous_xy = self._prev_track_xy.get(track_id)
            if previous is None or previous_xy is None:
                continue

            current_xy = (float(track.x), float(track.y))
            shifted = previous + (np.array(current_xy) - np.array(previous_xy))
            self._prev_predictions[track_id] = shifted.copy()
            self._prev_track_xy[track_id] = current_xy

            points = [
                TrajectoryPoint(
                    t_sec=(idx + 1) * dt,
                    x=float(shifted[idx, 0]),
                    y=float(shifted[idx, 1]),
                )
                for idx in range(len(shifted))
            ]
            trajectories.append(
                PredictedTrajectory(
                    track_id=int(track_id),
                    points=points,
                    confidence=1.0,
                    model_name="srlstm_cached",
                )
            )
        return trajectories

    def _sync_predictor_buffer_from_tracks(self, tracks: list[TrackedPedestrian]) -> None:
        """Use tracker history because it is already expressed in the current ego frame."""
        obs_length = int(getattr(self.predictor, "obs_length", 0))
        if obs_length <= 0:
            return

        for track in tracks:
            history_xy = [(float(point.x), float(point.y)) for point in track.history]
            current_xy = (float(track.x), float(track.y))
            if history_xy and np.linalg.norm(np.array(history_xy[-1]) - np.array(current_xy)) < 1e-3:
                seed = history_xy[-obs_length:-1]
            else:
                seed = history_xy[-(obs_length - 1):] if obs_length > 1 else []
            self.predictor.obs_buffer[track.track_id] = list(seed)

    def _sync_sampled_predictor_buffer(
        self,
        tracks: list[TrackedPedestrian],
        timestamp_sec: float,
    ) -> bool:
        """Keep SR-LSTM observations at the model's expected sampling interval."""
        obs_length = int(getattr(self.predictor, "obs_length", 0))
        sample_interval_sec = float(getattr(self.predictor, "dt", 0.0))
        if obs_length <= 0 or sample_interval_sec <= 0.0:
            return False

        min_interval_sec = sample_interval_sec * 0.95
        updated = False

        for track in tracks:
            track_id = track.track_id
            current_sample = (timestamp_sec, float(track.x), float(track.y))
            samples = self._sample_buffer.get(track_id)
            if not samples:
                samples = self._seed_sample_buffer_from_history(track, timestamp_sec, sample_interval_sec)
                if samples:
                    updated = True

            if samples and timestamp_sec < samples[-1][0] - 1e-3:
                samples = [current_sample]
                updated = True
            elif not samples:
                samples = [current_sample]
                updated = True
            elif timestamp_sec - samples[-1][0] >= min_interval_sec:
                samples.append(current_sample)
                updated = True

            samples = samples[-obs_length:]
            self._sample_buffer[track_id] = samples
            self.predictor.obs_buffer[track_id] = [(x, y) for _, x, y in samples]

        return updated

    def _seed_sample_buffer_from_history(
        self,
        track: TrackedPedestrian,
        timestamp_sec: float,
        sample_interval_sec: float,
    ) -> list[tuple[float, float, float]]:
        history = [
            (float(point.timestamp_sec), float(point.x), float(point.y))
            for point in track.history
            if point.timestamp_sec <= timestamp_sec + 1e-6
        ]
        current_sample = (timestamp_sec, float(track.x), float(track.y))
        if not history or abs(history[-1][0] - timestamp_sec) > 1e-3:
            history.append(current_sample)

        selected: list[tuple[float, float, float]] = []
        next_timestamp = float("inf")
        min_interval_sec = sample_interval_sec * 0.95
        obs_length = int(getattr(self.predictor, "obs_length", 0))
        for sample in reversed(history):
            if next_timestamp == float("inf") or next_timestamp - sample[0] >= min_interval_sec:
                selected.append(sample)
                next_timestamp = sample[0]
                if len(selected) >= obs_length:
                    break

        return list(reversed(selected))

    def _apply_ego_motion_to_cached_state(
        self,
        ego_motion_delta: tuple[float, float, float] | None,
    ) -> None:
        if ego_motion_delta is None:
            return

        dx_m, dy_m, dyaw_rad = ego_motion_delta
        if abs(dx_m) < 1e-6 and abs(dy_m) < 1e-6 and abs(dyaw_rad) < 1e-7:
            return

        for track_id, samples in list(self._sample_buffer.items()):
            self._sample_buffer[track_id] = [
                (timestamp, *self._transform_xy(x, y, dx_m, dy_m, dyaw_rad))
                for timestamp, x, y in samples
            ]

        for track_id, prediction in list(self._prev_predictions.items()):
            self._prev_predictions[track_id] = self._transform_xy_array(
                prediction,
                dx_m,
                dy_m,
                dyaw_rad,
            )

        for track_id, (x, y) in list(self._prev_track_xy.items()):
            self._prev_track_xy[track_id] = self._transform_xy(x, y, dx_m, dy_m, dyaw_rad)

        for track_id, points in list(self.predictor.obs_buffer.items()):
            self.predictor.obs_buffer[track_id] = [
                self._transform_xy(x, y, dx_m, dy_m, dyaw_rad)
                for x, y in points
            ]

        for track_id, prediction in list(getattr(self.predictor, "last_predictions", {}).items()):
            self.predictor.last_predictions[track_id] = self._transform_xy_array(
                prediction,
                dx_m,
                dy_m,
                dyaw_rad,
            )

    @staticmethod
    def _transform_xy(
        x: float,
        y: float,
        dx_m: float,
        dy_m: float,
        dyaw_rad: float,
    ) -> tuple[float, float]:
        translated_x = x - dx_m
        translated_y = y - dy_m
        cos_yaw = np.cos(-dyaw_rad)
        sin_yaw = np.sin(-dyaw_rad)
        return (
            float((cos_yaw * translated_x) - (sin_yaw * translated_y)),
            float((sin_yaw * translated_x) + (cos_yaw * translated_y)),
        )

    @classmethod
    def _transform_xy_array(
        cls,
        points: np.ndarray,
        dx_m: float,
        dy_m: float,
        dyaw_rad: float,
    ) -> np.ndarray:
        if len(points) == 0:
            return points.copy()
        translated = points.astype(float, copy=True)
        translated[:, 0] -= dx_m
        translated[:, 1] -= dy_m
        cos_yaw = np.cos(-dyaw_rad)
        sin_yaw = np.sin(-dyaw_rad)
        x = translated[:, 0].copy()
        y = translated[:, 1].copy()
        translated[:, 0] = (cos_yaw * x) - (sin_yaw * y)
        translated[:, 1] = (sin_yaw * x) + (cos_yaw * y)
        return translated

    def _purge_stale_state(self, active_ids: set[int]) -> None:
        for tid in [tid for tid in self._prev_predictions if tid not in active_ids]:
            del self._prev_predictions[tid]
            self._prev_track_xy.pop(tid, None)
        for tid in [tid for tid in self._sample_buffer if tid not in active_ids]:
            del self._sample_buffer[tid]
        for tid in [tid for tid in self.predictor.obs_buffer if tid not in active_ids]:
            self.predictor.obs_buffer.pop(tid, None)
            self.predictor.last_predictions.pop(tid, None)
            self.predictor.last_ttc.pop(tid, None)

    def _smooth_trajectory(
        self,
        track_id: int,
        raw_pred: np.ndarray,
        dt: float,
        current_xy: tuple[float, float] | None = None,
    ) -> np.ndarray:
        """Apply velocity clamping then EMA smoothing to a raw prediction."""
        clamped = self._clamp_velocity(raw_pred, dt)

        prev = self._prev_predictions.get(track_id)
        prev_track_xy = self._prev_track_xy.get(track_id)
        if prev is not None and prev_track_xy is not None and current_xy is not None:
            prev = prev + (np.array(current_xy) - np.array(prev_track_xy))
        if prev is not None and prev.shape == clamped.shape:
            alpha = self.smooth_alpha
            smoothed = alpha * clamped + (1.0 - alpha) * prev
        else:
            smoothed = clamped

        self._prev_predictions[track_id] = smoothed.copy()
        if current_xy is not None:
            self._prev_track_xy[track_id] = current_xy
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
