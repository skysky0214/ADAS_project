from __future__ import annotations

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from app.control.ttc_warning import StaticObstacleObservation, TTCWarning
from app.core.domain_types import PredictedTrajectory, TrackedPedestrian
from app.ego_motion import (
    DEFAULT_STEER_RATIO,
    EgoMotionDelta,
)


def build_tracking_marker_array(
    frame_id: str,
    timestamp,
    tracks: list[TrackedPedestrian],
    trajectories: list[PredictedTrajectory],
    warnings: list[TTCWarning],
    history_tail: int,
    static_obstacle: StaticObstacleObservation | None = None,
    ego_delta: EgoMotionDelta | None = None,
    ego_compensation_enabled: bool = False,
    ego_wheelbase_m: float = 2.9,
    ego_steer_ratio: float = DEFAULT_STEER_RATIO,
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
    warning_by_track = {warning.track_id: warning for warning in warnings if warning.track_id >= 0}

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
    _append_static_obstacle_markers(
        marker_array=marker_array,
        frame_id=frame_id,
        timestamp=timestamp,
        static_obstacle=static_obstacle,
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
                f"L{warning.level} TTC {_format_ttc(warning.min_ttc_sec)} "
                f"a={warning.target_accel_mps2:.1f}"
            )
            _set_marker_color(warning_marker, _warning_level_rgba(warning.level))
            marker_array.markers.append(warning_marker)

            normal_marker = _base_marker(
                frame_id,
                timestamp,
                "ttc_normal_warnings",
                60_000 + track.track_id,
                Marker.TEXT_VIEW_FACING,
            )
            normal_marker.pose.position.x = float(track.x)
            normal_marker.pose.position.y = float(track.y)
            normal_marker.pose.position.z = float(track.z + (track.dz or 1.7) + 1.65)
            normal_marker.scale.z = 0.52
            normal_marker.text = f"BASE L{warning.normal_level}"
            _set_marker_color(normal_marker, _warning_level_rgba(warning.normal_level))
            marker_array.markers.append(normal_marker)

    # Render static obstacle warnings
    for warning in warnings:
        if warning.track_id == -99 and (warning.level > 0 or warning.normal_level > 0):
            marker_x = float(static_obstacle.centroid_x) if static_obstacle is not None else float(warning.distance_m + vehicle_front_m)
            marker_y = float(static_obstacle.centroid_y) if static_obstacle is not None else 0.0
            marker_z = float(static_obstacle.z_max + 0.8) if static_obstacle is not None else 0.8
            # TEXT marker for warning message
            text_marker = _base_marker(
                frame_id,
                timestamp,
                "static_ttc_warnings",
                50_000,
                Marker.TEXT_VIEW_FACING,
            )
            text_marker.pose.position.x = marker_x
            text_marker.pose.position.y = marker_y
            text_marker.pose.position.z = marker_z
            text_marker.scale.z = 0.7
            text_marker.text = (
                f"[STATIC] L{warning.level} BASE L{warning.normal_level} "
                f"TTC {_format_ttc(warning.min_ttc_sec)} "
                f"front_dist={warning.distance_m:.1f}m"
            )
            _set_marker_color(text_marker, _warning_level_rgba(max(warning.level, warning.normal_level)))
            marker_array.markers.append(text_marker)

            # CUBE marker to highlight the static obstacle warning zone
            box_marker = _base_marker(
                frame_id,
                timestamp,
                "static_obstacle_box",
                50_001,
                Marker.CUBE,
            )
            if static_obstacle is not None:
                box_marker.pose.position.x = float((static_obstacle.x_min + static_obstacle.x_max) * 0.5)
                box_marker.pose.position.y = float((static_obstacle.y_min + static_obstacle.y_max) * 0.5)
                box_marker.pose.position.z = float((static_obstacle.z_min + static_obstacle.z_max) * 0.5)
                box_marker.scale.x = max(float(static_obstacle.x_max - static_obstacle.x_min), 0.30)
                box_marker.scale.y = max(float(static_obstacle.y_max - static_obstacle.y_min), 0.30)
                box_marker.scale.z = max(float(static_obstacle.z_max - static_obstacle.z_min), 0.30)
            else:
                box_marker.pose.position.x = marker_x
                box_marker.pose.position.y = marker_y
                box_marker.pose.position.z = 0.0
                box_marker.scale.x = 1.0
                box_marker.scale.y = 2.0
                box_marker.scale.z = 1.0

            # Semi-transparent coloring based on danger level
            base_rgba = _warning_level_rgba(max(warning.level, warning.normal_level))
            alpha = {1: 0.35, 2: 0.45, 3: 0.55}.get(max(warning.level, warning.normal_level), 0.2)
            rgba = (base_rgba[0], base_rgba[1], base_rgba[2], alpha)
            _set_marker_color(box_marker, rgba)
            marker_array.markers.append(box_marker)

    return marker_array


def _format_ttc(ttc_sec: float) -> str:
    if not math.isfinite(ttc_sec):
        return "inf"
    return f"{ttc_sec:.2f}s"


def _warning_level_rgba(level: int) -> tuple[float, float, float, float]:
    return {
        0: (0.0, 1.0, 0.35, 1.0),
        1: (1.0, 1.0, 0.0, 1.0),
        2: (1.0, 0.55, 0.0, 1.0),
        3: (1.0, 0.0, 0.0, 1.0),
    }.get(level, (1.0, 1.0, 1.0, 1.0))


def _append_static_obstacle_markers(
    marker_array: MarkerArray,
    frame_id: str,
    timestamp,
    static_obstacle: StaticObstacleObservation | None,
) -> None:
    if static_obstacle is None:
        return

    rgba = _static_obstacle_rgba(static_obstacle)
    x_center = (static_obstacle.x_min + static_obstacle.x_max) * 0.5
    y_center = (static_obstacle.y_min + static_obstacle.y_max) * 0.5
    z_center = (static_obstacle.z_min + static_obstacle.z_max) * 0.5

    box = _base_marker(frame_id, timestamp, "static_obstacle_candidate", 1, Marker.CUBE)
    box.pose.position.x = float(x_center)
    box.pose.position.y = float(y_center)
    box.pose.position.z = float(z_center)
    box.scale.x = max(float(static_obstacle.x_max - static_obstacle.x_min), 0.30)
    box.scale.y = max(float(static_obstacle.y_max - static_obstacle.y_min), 0.30)
    box.scale.z = max(float(static_obstacle.z_max - static_obstacle.z_min), 0.30)
    _set_marker_color(box, (rgba[0], rgba[1], rgba[2], 0.28 if static_obstacle.level == 0 else 0.48))
    marker_array.markers.append(box)

    center = _base_marker(frame_id, timestamp, "static_obstacle_candidate", 2, Marker.SPHERE)
    center.pose.position.x = float(static_obstacle.centroid_x)
    center.pose.position.y = float(static_obstacle.centroid_y)
    center.pose.position.z = float(static_obstacle.centroid_z)
    center.scale.x = 0.35
    center.scale.y = 0.35
    center.scale.z = 0.35
    _set_marker_color(center, (rgba[0], rgba[1], rgba[2], 0.95))
    marker_array.markers.append(center)

    ttc_text = "SUPP" if static_obstacle.suppressed_low_speed else f"TTC={static_obstacle.ttc_sec:.2f}s"
    label = _base_marker(frame_id, timestamp, "static_obstacle_candidate", 3, Marker.TEXT_VIEW_FACING)
    label.pose.position.x = float(static_obstacle.centroid_x)
    label.pose.position.y = float(static_obstacle.centroid_y)
    label.pose.position.z = float(static_obstacle.z_max + 0.8)
    label.scale.z = 0.55
    label.text = (
        f"STATIC CLOUD pts={static_obstacle.point_count} "
        f"front_dist={static_obstacle.distance_m:.1f}m {ttc_text} L{static_obstacle.level}"
    )
    _set_marker_color(label, (rgba[0], rgba[1], rgba[2], 1.0))
    marker_array.markers.append(label)


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
    accel_pct = max(0.0, min(1.0, float(delta.accelerator_pedal))) * 100.0
    brake_state = "ON" if delta.brake_pressed or delta.brake_lights else "OFF"
    accel_state = "ON" if delta.accelerator_pressed else "OFF"
    status.text = (
        f"EGO COMP {state} | v={delta.speed_mps:.2f} m/s "
        f"steer={delta.steering_deg:.1f} deg\n"
        f"ACCEL={accel_state} {accel_pct:.0f}% | BRAKE={brake_state} | "
        f"d=({delta.dx_m:.3f},{delta.dy_m:.3f}) yaw={delta.dyaw_rad:.4f} "
        f"reset={delta.reset}"
    )
    _set_marker_color(status, status_color)
    marker_array.markers.append(status)


def _static_obstacle_rgba(static_obstacle: StaticObstacleObservation) -> tuple[float, float, float, float]:
    if static_obstacle.level == 3:
        return (1.0, 0.0, 0.0, 1.0)
    if static_obstacle.level == 2:
        return (1.0, 0.55, 0.0, 1.0)
    if static_obstacle.level == 1:
        return (1.0, 1.0, 0.0, 1.0)
    if static_obstacle.suppressed_low_speed:
        return (0.55, 0.75, 1.0, 1.0)
    return (0.0, 0.95, 1.0, 1.0)


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
    road_angle_rad = math.radians(steering_deg) / max(steer_ratio, 1e-6)
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
