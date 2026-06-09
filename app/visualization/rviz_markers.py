from __future__ import annotations

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from app.control.ttc_warning import TTCWarning
from app.core.domain_types import PredictedTrajectory, TrackedPedestrian
from app.ego_motion import EgoMotionDelta


def build_tracking_marker_array(
    frame_id: str,
    timestamp,
    tracks: list[TrackedPedestrian],
    trajectories: list[PredictedTrajectory],
    warnings: list[TTCWarning],
    history_tail: int,
    ego_delta: EgoMotionDelta | None = None,
    ego_compensation_enabled: bool = False,
    ego_wheelbase_m: float = 2.9,
    ego_steer_ratio: float = 16.0,
    vehicle_front_m: float = 2.4,
    vehicle_rear_m: float = 2.1,
    vehicle_side_m: float = 1.0,
    ego_prediction_horizon_sec: float = 3.0,
    ego_prediction_step_sec: float = 0.2,
) -> MarkerArray:
    marker_array = MarkerArray()

    delete_marker = Marker()
    delete_marker.header.frame_id = frame_id
    delete_marker.header.stamp = timestamp
    delete_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_marker)

    prediction_by_track = {trajectory.track_id: trajectory for trajectory in trajectories}
    warning_by_track = {warning.track_id: warning for warning in warnings if warning.level > 0}

    _append_ego_markers(
        marker_array=marker_array,
        frame_id=frame_id,
        timestamp=timestamp,
        ego_delta=ego_delta,
        ego_compensation_enabled=ego_compensation_enabled,
        wheelbase_m=ego_wheelbase_m,
        steer_ratio=ego_steer_ratio,
        vehicle_front_m=vehicle_front_m,
        vehicle_rear_m=vehicle_rear_m,
        vehicle_side_m=vehicle_side_m,
        horizon_sec=ego_prediction_horizon_sec,
        step_sec=ego_prediction_step_sec,
    )

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

    return marker_array


def _append_ego_markers(
    marker_array: MarkerArray,
    frame_id: str,
    timestamp,
    ego_delta: EgoMotionDelta | None,
    ego_compensation_enabled: bool,
    wheelbase_m: float,
    steer_ratio: float,
    vehicle_front_m: float,
    vehicle_rear_m: float,
    vehicle_side_m: float,
    horizon_sec: float,
    step_sec: float,
) -> None:
    delta = ego_delta or EgoMotionDelta()
    status_color = (0.0, 0.95, 0.35, 1.0) if ego_compensation_enabled and delta.valid else (1.0, 0.75, 0.0, 1.0)

    footprint = _base_marker(frame_id, timestamp, "ego_vehicle", 1, Marker.CUBE)
    footprint.pose.position.x = float((vehicle_front_m - vehicle_rear_m) * 0.5)
    footprint.pose.position.y = 0.0
    footprint.pose.position.z = 0.08
    footprint.scale.x = float(vehicle_front_m + vehicle_rear_m)
    footprint.scale.y = float(vehicle_side_m * 2.0)
    footprint.scale.z = 0.16
    _set_marker_color(footprint, (0.2, 0.6, 1.0, 0.22))
    marker_array.markers.append(footprint)

    center_line = _base_marker(frame_id, timestamp, "ego_vehicle", 2, Marker.LINE_STRIP)
    center_line.scale.x = 0.05
    center_line.points = [_point(-vehicle_rear_m, 0.0, 0.22), _point(vehicle_front_m, 0.0, 0.22)]
    _set_marker_color(center_line, (0.2, 0.6, 1.0, 0.9))
    marker_array.markers.append(center_line)

    heading = _base_marker(frame_id, timestamp, "ego_vehicle", 3, Marker.ARROW)
    heading.points = [_point(0.0, 0.0, 0.35), _point(vehicle_front_m, 0.0, 0.35)]
    heading.scale.x = 0.12
    heading.scale.y = 0.28
    heading.scale.z = 0.25
    _set_marker_color(heading, status_color)
    marker_array.markers.append(heading)

    path_points = _predict_ego_path(
        speed_mps=delta.speed_mps,
        steering_deg=delta.steering_deg,
        wheelbase_m=wheelbase_m,
        steer_ratio=steer_ratio,
        horizon_sec=horizon_sec,
        step_sec=step_sec,
    )

    path = _base_marker(frame_id, timestamp, "ego_predicted_path", 1, Marker.LINE_STRIP)
    path.scale.x = 0.12
    path.points = [_point(x, y, 0.45) for x, y, _yaw in path_points]
    _set_marker_color(path, (0.0, 1.0, 0.35, 0.95) if delta.speed_mps >= 0.0 else (1.0, 0.45, 0.0, 0.95))
    marker_array.markers.append(path)

    end_x, end_y, end_yaw = path_points[-1]
    future = _base_marker(frame_id, timestamp, "ego_future_pose", 1, Marker.CUBE)
    future.pose.position.x = float(end_x + math.cos(end_yaw) * (vehicle_front_m - vehicle_rear_m) * 0.5)
    future.pose.position.y = float(end_y + math.sin(end_yaw) * (vehicle_front_m - vehicle_rear_m) * 0.5)
    future.pose.position.z = 0.12
    _yaw_to_quaternion(future, end_yaw)
    future.scale.x = float(vehicle_front_m + vehicle_rear_m)
    future.scale.y = float(vehicle_side_m * 2.0)
    future.scale.z = 0.24
    _set_marker_color(future, (0.0, 1.0, 0.35, 0.22))
    marker_array.markers.append(future)

    status = _base_marker(frame_id, timestamp, "ego_status", 1, Marker.TEXT_VIEW_FACING)
    status.pose.position.x = 0.0
    status.pose.position.y = -vehicle_side_m - 0.8
    status.pose.position.z = 1.6
    status.scale.z = 0.5
    state = "ON" if ego_compensation_enabled and delta.valid else "WAIT"
    status.text = (
        f"EGO COMP {state} | v={delta.speed_mps:.2f} m/s "
        f"steer={delta.steering_deg:.1f} deg | "
        f"d=({delta.dx_m:.3f},{delta.dy_m:.3f}) yaw={delta.dyaw_rad:.4f} "
        f"reset={delta.reset}"
    )
    _set_marker_color(status, status_color)
    marker_array.markers.append(status)


def _predict_ego_path(
    speed_mps: float,
    steering_deg: float,
    wheelbase_m: float,
    steer_ratio: float,
    horizon_sec: float,
    step_sec: float,
) -> list[tuple[float, float, float]]:
    points = [(0.0, 0.0, 0.0)]
    step = max(step_sec, 0.05)
    steps = max(1, int(max(horizon_sec, step) / step))
    road_angle_rad = math.radians(steering_deg / max(steer_ratio, 1e-6))
    yaw_rate = speed_mps / max(wheelbase_m, 1e-6) * math.tan(road_angle_rad)
    x = 0.0
    y = 0.0
    yaw = 0.0
    for _ in range(steps):
        dyaw = yaw_rate * step
        mid_yaw = yaw + 0.5 * dyaw
        x += speed_mps * math.cos(mid_yaw) * step
        y += speed_mps * math.sin(mid_yaw) * step
        yaw += dyaw
        points.append((x, y, yaw))
    return points


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
