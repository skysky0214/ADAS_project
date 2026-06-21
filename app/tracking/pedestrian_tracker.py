from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

from app.core.domain_types import HistoryPoint, PedestrianDetection, TrackedPedestrian
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

    def reset(self) -> None:
        self._tracks.clear()

    def apply_ego_motion(self, dx_m: float, dy_m: float, dyaw_rad: float, reset: bool = False) -> None:
        """Move existing tracks from the previous ego frame into the current one.

        New detections are already expressed in the current LiDAR/ego frame. Before
        matching, old tracks and history must be transformed by the vehicle motion:

          p_current = R(-dyaw) @ (p_previous - [dx, dy])
        """
        if reset:
            self.reset()
            return
        if not self._tracks:
            return
        if abs(dx_m) < 1e-6 and abs(dy_m) < 1e-6 and abs(dyaw_rad) < 1e-7:
            return

        cos_yaw = math.cos(-dyaw_rad)
        sin_yaw = math.sin(-dyaw_rad)

        def transform_xy(x: float, y: float) -> tuple[float, float]:
            translated_x = x - dx_m
            translated_y = y - dy_m
            return (
                (cos_yaw * translated_x) - (sin_yaw * translated_y),
                (sin_yaw * translated_x) + (cos_yaw * translated_y),
            )

        updated: Dict[int, _TrackState] = {}
        for track_id, state in self._tracks.items():
            track = state.track
            x, y = transform_xy(track.x, track.y)
            vx = (cos_yaw * track.vx) - (sin_yaw * track.vy)
            vy = (sin_yaw * track.vx) + (cos_yaw * track.vy)
            history = []
            for point in track.history:
                history_x, history_y = transform_xy(point.x, point.y)
                history.append(
                    HistoryPoint(
                        frame_id=point.frame_id,
                        timestamp_sec=point.timestamp_sec,
                        x=history_x,
                        y=history_y,
                    )
                )
            heading = track.heading - dyaw_rad if track.heading is not None else None
            updated[track_id] = _TrackState(
                track=TrackedPedestrian(
                    track_id=track.track_id,
                    x=x,
                    y=y,
                    z=track.z,
                    vx=vx,
                    vy=vy,
                    missed=track.missed,
                    score=track.score,
                    dx=track.dx,
                    dy=track.dy,
                    dz=track.dz,
                    heading=heading,
                    point_max_distance_m=track.point_max_distance_m,
                    point_count=track.point_count,
                    history=history,
                )
            )
        self._tracks = updated

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
                    z=det.z,
                    score=det.score,
                    dx=det.dx,
                    dy=det.dy,
                    dz=det.dz,
                    heading=det.heading,
                    point_max_distance_m=det.point_max_distance_m,
                    point_count=det.point_count,
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
                z=det.z,
                vx=vx,
                vy=vy,
                missed=0,
                score=det.score,
                dx=det.dx,
                dy=det.dy,
                dz=det.dz,
                heading=det.heading,
                point_max_distance_m=det.point_max_distance_m,
                point_count=det.point_count,
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
                        z=state.track.z,
                        vx=state.track.vx,
                        vy=state.track.vy,
                        missed=missed,
                        score=state.track.score,
                        dx=state.track.dx,
                        dy=state.track.dy,
                        dz=state.track.dz,
                        heading=state.track.heading,
                        point_max_distance_m=state.track.point_max_distance_m,
                        point_count=state.track.point_count,
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
