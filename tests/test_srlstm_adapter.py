from __future__ import annotations

from collections import defaultdict

import numpy as np

from app.core.domain_types import HistoryPoint, TrackedPedestrian
from app.prediction.adapters.srlstm_predictor import SRLSTMPredictionModel


class _FakePredictor:
    def __init__(self) -> None:
        self.obs_length = 4
        self.dt = 0.4
        self.obs_buffer = defaultdict(list)
        self.last_predictions = {}
        self.last_ttc = {}


def _model_without_checkpoint() -> SRLSTMPredictionModel:
    model = object.__new__(SRLSTMPredictionModel)
    model.predictor = _FakePredictor()
    model.smooth_alpha = 0.4
    model.max_pedestrian_speed = 3.0
    model._prev_predictions = {}
    model._prev_track_xy = {}
    model._sample_buffer = {}
    return model


def test_srlstm_buffer_uses_compensated_tracker_history_without_dup_current() -> None:
    model = _model_without_checkpoint()
    track = TrackedPedestrian(
        track_id=7,
        x=8.5,
        y=0.0,
        history=[
            HistoryPoint(frame_id=1, timestamp_sec=1.0, x=10.0, y=0.0),
            HistoryPoint(frame_id=2, timestamp_sec=1.1, x=9.5, y=0.0),
            HistoryPoint(frame_id=3, timestamp_sec=1.2, x=9.0, y=0.0),
            HistoryPoint(frame_id=4, timestamp_sec=1.3, x=8.5, y=0.0),
        ],
    )

    model._sync_predictor_buffer_from_tracks([track])

    assert model.predictor.obs_buffer[7] == [(10.0, 0.0), (9.5, 0.0), (9.0, 0.0)]


def test_srlstm_smoothing_shifts_previous_prediction_to_current_ego_frame() -> None:
    model = _model_without_checkpoint()
    model.smooth_alpha = 0.0
    model._prev_predictions[1] = np.array([[10.0, 0.0], [11.0, 0.0]])
    model._prev_track_xy[1] = (10.0, 0.0)
    raw_pred = np.array([[9.0, 0.0], [10.0, 0.0]])

    smoothed = model._smooth_trajectory(1, raw_pred, dt=0.4, current_xy=(9.0, 0.0))

    assert np.allclose(smoothed, np.array([[9.0, 0.0], [10.0, 0.0]]))


def test_srlstm_sample_buffer_uses_timestamp_interval() -> None:
    model = _model_without_checkpoint()

    first = TrackedPedestrian(
        track_id=3,
        x=10.0,
        y=0.0,
        history=[
            HistoryPoint(frame_id=1, timestamp_sec=1.0, x=10.0, y=0.0),
        ],
    )
    assert model._sync_sampled_predictor_buffer([first], timestamp_sec=1.0)
    assert model.predictor.obs_buffer[3] == [(10.0, 0.0)]

    too_soon = TrackedPedestrian(
        track_id=3,
        x=9.9,
        y=0.0,
        history=[
            HistoryPoint(frame_id=2, timestamp_sec=1.05, x=9.9, y=0.0),
        ],
    )
    assert not model._sync_sampled_predictor_buffer([too_soon], timestamp_sec=1.05)
    assert model.predictor.obs_buffer[3] == [(10.0, 0.0)]

    due = TrackedPedestrian(
        track_id=3,
        x=9.2,
        y=0.0,
        history=[
            HistoryPoint(frame_id=3, timestamp_sec=1.4, x=9.2, y=0.0),
        ],
    )
    assert model._sync_sampled_predictor_buffer([due], timestamp_sec=1.4)
    assert model.predictor.obs_buffer[3] == [(10.0, 0.0), (9.2, 0.0)]


def test_srlstm_cached_state_is_ego_motion_compensated() -> None:
    model = _model_without_checkpoint()
    model._sample_buffer[4] = [
        (1.0, 10.0, 0.5),
        (1.4, 10.2, 0.5),
    ]
    model._prev_predictions[4] = np.array([[10.5, 0.5], [11.0, 0.5]])
    model._prev_track_xy[4] = (10.2, 0.5)
    model.predictor.obs_buffer[4] = [(10.0, 0.5), (10.2, 0.5)]
    model.predictor.last_predictions[4] = np.array([[10.5, 0.5], [11.0, 0.5]])

    model._apply_ego_motion_to_cached_state((1.0, 0.0, 0.0))

    assert model._sample_buffer[4] == [
        (1.0, 9.0, 0.5),
        (1.4, 9.2, 0.5),
    ]
    assert np.allclose(model._prev_predictions[4], np.array([[9.5, 0.5], [10.0, 0.5]]))
    assert model._prev_track_xy[4] == (9.2, 0.5)
    assert model.predictor.obs_buffer[4] == [(9.0, 0.5), (9.2, 0.5)]
    assert np.allclose(model.predictor.last_predictions[4], np.array([[9.5, 0.5], [10.0, 0.5]]))
