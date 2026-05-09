from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

from app.domain_types import HistoryPoint, PedestrianDetection, TrackedPedestrian
from app.tracking.matcher import euclidean_distance


@dataclass
class _TrackState:
    track: TrackedPedestrian


class PedestrianTracker:
    """
    Real-time pedestrian tracker.

    Input: current-frame pedestrian detections without IDs
    Output: tracked pedestrians with persistent IDs and xy history
    """

    def __init__(
        self,
        match_distance: float,
        reconnect_distance: float,
        max_missed: int,
        history_size: int,
    ):
        self.match_distance = match_distance
        self.reconnect_distance = reconnect_distance
        self.max_missed = max_missed
        self.history_size = history_size
        self._next_track_id = 1
        self._tracks: Dict[int, _TrackState] = {}

    def update(
        self,
        frame_id: int,
        timestamp_sec: float,
        detections: List[PedestrianDetection],
    ) -> List[TrackedPedestrian]:
        updated: Dict[int, _TrackState] = {}
        used_track_ids = set()

        for det in detections:
            matched_id = self._match(det, used_track_ids, timestamp_sec)

            if matched_id is None:
                track = TrackedPedestrian(
                    track_id=self._next_track_id,
                    x=det.x,
                    y=det.y,
                    history=[
                        HistoryPoint(
                            frame_id=frame_id,
                            timestamp_sec=timestamp_sec,
                            x=det.x,
                            y=det.y,
                        )
                    ],
                )
                updated[self._next_track_id] = _TrackState(track=track)
                used_track_ids.add(self._next_track_id)
                self._next_track_id += 1
                continue

            prev = self._tracks[matched_id].track
            prev_last = prev.history[-1]
            dt = max(timestamp_sec - prev_last.timestamp_sec, 1e-6)
            vx = (det.x - prev.x) / dt
            vy = (det.y - prev.y) / dt
            history = (
                prev.history
                + [
                    HistoryPoint(
                        frame_id=frame_id,
                        timestamp_sec=timestamp_sec,
                        x=det.x,
                        y=det.y,
                    )
                ]
            )[-self.history_size :]
            track = TrackedPedestrian(
                track_id=matched_id,
                x=det.x,
                y=det.y,
                vx=vx,
                vy=vy,
                missed=0,
                history=history,
            )
            updated[matched_id] = _TrackState(track=track)
            used_track_ids.add(matched_id)

        for track_id, state in self._tracks.items():
            if track_id in updated:
                continue
            missed = state.track.missed + 1
            if missed <= self.max_missed:
                updated[track_id] = _TrackState(
                    track=TrackedPedestrian(
                        track_id=state.track.track_id,
                        x=state.track.x,
                        y=state.track.y,
                        vx=state.track.vx,
                        vy=state.track.vy,
                        missed=missed,
                        history=state.track.history,
                    )
                )

        self._tracks = updated
        return [state.track for state in self._tracks.values()]

    def _match(
        self,
        det: PedestrianDetection,
        used_track_ids: set[int],
        timestamp_sec: float,
    ) -> int | None:
        best_id = None
        best_distance = float("inf")

        for track_id, state in self._tracks.items():
            if track_id in used_track_ids:
                continue
            predicted_x, predicted_y = self._predict_position(state.track, timestamp_sec)
            distance = euclidean_distance(det, predicted_x, predicted_y)
            limit = self.match_distance if state.track.missed == 0 else self.reconnect_distance
            if distance <= limit and distance < best_distance:
                best_distance = distance
                best_id = track_id

        return best_id

    @staticmethod
    def _predict_position(track: TrackedPedestrian, timestamp_sec: float) -> tuple[float, float]:
        if not track.history:
            return track.x, track.y

        last_time = track.history[-1].timestamp_sec
        dt = max(timestamp_sec - last_time, 0.0)
        predicted_x = track.x + (track.vx * dt)
        predicted_y = track.y + (track.vy * dt)
        return predicted_x, predicted_y
