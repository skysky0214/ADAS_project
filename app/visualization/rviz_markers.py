from __future__ import annotations

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from app.control.ttc_warning import TTCWarning
from app.core.domain_types import PredictedTrajectory, TrackedPedestrian


def build_tracking_marker_array(
    frame_id: str,
    timestamp,
    tracks: list[TrackedPedestrian],
    trajectories: list[PredictedTrajectory],
    warnings: list[TTCWarning],
    history_tail: int,
) -> MarkerArray:
    marker_array = MarkerArray()

    delete_marker = Marker()
    delete_marker.header.frame_id = frame_id
    delete_marker.header.stamp = timestamp
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)

    prediction_by_track = {trajectory.track_id: trajectory for trajectory in trajectories}
    warning_by_track = {warning.track_id: warning for warning in warnings if warning.level > 0}

    for track in tracks:
        box = _base_marker(frame_id, timestamp, "tracked_boxes", track.track_id, Marker.CUBE)
        box.pose.position.x = float(track.x)
        box.pose.position.y = float(track.y)
        box.pose.position.z = float(track.z)
        _yaw_to_quaternion(box, float(track.heading or 0.0))
        box.scale.x = float(track.dx or 0.8)
        box.scale.y = float(track.dy or 0.8)
        box.scale.z = float(track.dz or 1.7)
        _set_marker_color(box, (0.1, 0.75, 1.0, 0.35 if track.missed else 0.7))
        marker_array.markers.append(box)

        text = _base_marker(frame_id, timestamp, "track_ids", 10_000 + track.track_id, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = float(track.x)
        text.pose.position.y = float(track.y)
        text.pose.position.z = float(track.z + (track.dz or 1.7) + 0.35)
        text.scale.z = 0.6
        text.text = f"ID {track.track_id}"
        _set_marker_color(text, (1.0, 1.0, 1.0, 1.0))
        marker_array.markers.append(text)

        history = track.history[-history_tail:] if history_tail > 0 else track.history
        if len(history) >= 2:
            history_marker = _base_marker(
                frame_id,
                timestamp,
                "track_history",
                20_000 + track.track_id,
                Marker.LINE_STRIP,
            )
            history_marker.scale.x = 0.08
            _set_marker_color(history_marker, (0.0, 1.0, 0.3, 0.9))
            history_marker.points = [_point(point.x, point.y, 0.15) for point in history]
            marker_array.markers.append(history_marker)

        trajectory = prediction_by_track.get(track.track_id)
        if trajectory is not None and trajectory.points:
            pred_marker = _base_marker(
                frame_id,
                timestamp,
                "predicted_paths",
                30_000 + track.track_id,
                Marker.LINE_STRIP,
            )
            pred_marker.scale.x = 0.08
            _set_marker_color(pred_marker, (1.0, 0.2, 0.2, 0.95))
            pred_marker.points = [_point(track.x, track.y, 0.25)]
            pred_marker.points.extend(_point(point.x, point.y, 0.25) for point in trajectory.points)
            marker_array.markers.append(pred_marker)

        warning = warning_by_track.get(track.track_id)
        if warning is not None:
            warning_marker = _base_marker(
                frame_id,
                timestamp,
                "ttc_warnings",
                40_000 + track.track_id,
                Marker.TEXT_VIEW_FACING,
            )
            warning_marker.pose.position.x = float(track.x)
            warning_marker.pose.position.y = float(track.y)
            warning_marker.pose.position.z = float(track.z + (track.dz or 1.7) + 1.0)
            warning_marker.scale.z = 0.65
            warning_marker.text = (
                f"L{warning.level} TTC {warning.min_ttc_sec:.2f}s "
                f"a={warning.target_accel_mps2:.1f}"
            )
            color = {
                1: (1.0, 1.0, 0.0, 1.0),
                2: (1.0, 0.55, 0.0, 1.0),
                3: (1.0, 0.0, 0.0, 1.0),
            }.get(warning.level, (1.0, 1.0, 1.0, 1.0))
            _set_marker_color(warning_marker, color)
            marker_array.markers.append(warning_marker)

    # Render static obstacle warnings
    for warning in warnings:
        if warning.track_id == -99 and warning.level > 0:
            # TEXT marker for warning message
            text_marker = _base_marker(
                frame_id,
                timestamp,
                "static_ttc_warnings",
                50_000,
                Marker.TEXT_VIEW_FACING,
            )
            text_marker.pose.position.x = float(warning.distance_m)
            text_marker.pose.position.y = 0.0
            text_marker.pose.position.z = 0.8
            text_marker.scale.z = 0.7
            text_marker.text = (
                f"[STATIC] L{warning.level} TTC {warning.min_ttc_sec:.2f}s "
                f"dist={warning.distance_m:.1f}m"
            )
            color = {
                1: (1.0, 1.0, 0.0, 1.0),
                2: (1.0, 0.55, 0.0, 1.0),
                3: (1.0, 0.0, 0.0, 1.0),
            }.get(warning.level, (1.0, 1.0, 1.0, 1.0))
            _set_marker_color(text_marker, color)
            marker_array.markers.append(text_marker)

            # CUBE marker to highlight the static obstacle warning zone
            box_marker = _base_marker(
                frame_id,
                timestamp,
                "static_obstacle_box",
                50_001,
                Marker.CUBE,
            )
            box_marker.pose.position.x = float(warning.distance_m)
            box_marker.pose.position.y = 0.0
            box_marker.pose.position.z = 0.0
            box_marker.scale.x = 1.0
            box_marker.scale.y = 2.0
            box_marker.scale.z = 1.0
            
            # Semi-transparent coloring based on danger level
            rgba = {
                1: (1.0, 1.0, 0.0, 0.35),
                2: (1.0, 0.55, 0.0, 0.45),
                3: (1.0, 0.0, 0.0, 0.55),
            }.get(warning.level, (1.0, 1.0, 1.0, 0.2))
            _set_marker_color(box_marker, rgba)
            marker_array.markers.append(box_marker)

    return marker_array


def _set_marker_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def _point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _base_marker(frame_id: str, timestamp, namespace: str, marker_id: int, marker_type: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = timestamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _yaw_to_quaternion(marker: Marker, yaw: float) -> None:
    marker.pose.orientation.z = math.sin(yaw / 2.0)
    marker.pose.orientation.w = math.cos(yaw / 2.0)

