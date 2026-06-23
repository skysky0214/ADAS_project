from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from app.core.config import DEFAULT_STATIC_ROI_Z_MAX_M, DEFAULT_STATIC_ROI_Z_MIN_M
from app.core.domain_types import PredictedTrajectory, TrackedPedestrian, TrajectoryPoint


@dataclass(frozen=True)
class TTCWarningConfig:
    level1_ttc_sec: float = 1.70
    level2_ttc_sec: float = 1.50
    level3_ttc_sec: float = 0.80
    prediction_dt_sec: float = 0.4
    ttc_horizon_sec: float = 6.0
    safety_radius_m: float = 1.0
    ego_speed_mps: float = 10.0
    ego_steering_deg: float = 0.0
    ego_wheelbase_m: float = 2.90
    ego_steer_ratio: float = 14.25
    vehicle_front_m: float = 2.40
    vehicle_rear_m: float = 2.10
    vehicle_side_m: float = 1.00
    low_speed_suppress_mps: float = 10.0 / 3.6
    perception_low_speed_suppress_mps: float = 5.0 / 3.6
    driver_brake_pressed: bool = False
    driver_accelerator_pressed: bool = False
    brake_ttc_scale: float = 0.70
    max_decel_mps2: float = -8.0
    max_jerk_mps3: float = 50.0
    roi_x_min: float = 2.5
    roi_x_max: float = 15.0
    roi_y_min: float = -1.1
    roi_y_max: float = 1.1
    roi_z_min: float = DEFAULT_STATIC_ROI_Z_MIN_M
    roi_z_max: float = DEFAULT_STATIC_ROI_Z_MAX_M
    static_obstacle_min_points: int = 15
    static_cluster_bin_m: float = 0.40
    static_cluster_min_cell_points: int = 3


@dataclass(frozen=True)
class TTCWarning:
    frame_id: int
    timestamp_sec: float
    track_id: int
    level: int
    label: str
    action: str
    color: str
    normal_level: int
    normal_label: str
    normal_action: str
    normal_color: str
    min_ttc_sec: float
    target_accel_mps2: float
    distance_m: float
    collision_time_sec: float


@dataclass(frozen=True)
class StaticObstacleObservation:
    frame_id: int
    timestamp_sec: float
    point_count: int
    level: int
    label: str
    suppressed_low_speed: bool
    distance_m: float
    ttc_sec: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    centroid_x: float
    centroid_y: float
    centroid_z: float


class TTCWarningAdapter:
    """Evaluate prediction trajectories and return warning/deceleration candidates.

    This adapter does not command the vehicle. It only produces a warning level and
    a target acceleration candidate for the future control interface.
    """

    def __init__(self, config: TTCWarningConfig | None = None):
        self.config = config or TTCWarningConfig()
        self.latest_static_obstacle: StaticObstacleObservation | None = None

    def evaluate(
        self,
        frame_id: int,
        timestamp_sec: float,
        tracked_objects: list[TrackedPedestrian],
        predicted_trajectories: list[PredictedTrajectory],
        points: np.ndarray | None = None,
    ) -> list[TTCWarning]:
        predicted_by_id = {trajectory.track_id: trajectory for trajectory in predicted_trajectories}
        warnings = []
        for track in tracked_objects:
            if track.missed > 0:
                continue
            trajectory = predicted_by_id.get(track.track_id)
            trajectory_points = self._trajectory_points_for_ttc(track, trajectory)
            ttc_result = self.compute_ttc(track, trajectory_points)
            warning_info = self.classify_warning(ttc_result.min_ttc_sec)
            normal_warning_info = self.classify_warning_normal(ttc_result.min_ttc_sec)
            accel = self.target_deceleration(ttc_result.min_ttc_sec)
            collision_time_sec = (
                timestamp_sec + ttc_result.min_ttc_sec
                if math.isfinite(ttc_result.min_ttc_sec)
                else float("inf")
            )
            warnings.append(
                TTCWarning(
                    frame_id=frame_id,
                    timestamp_sec=timestamp_sec,
                    track_id=track.track_id,
                    level=warning_info["level"],
                    label=warning_info["label"],
                    action=warning_info["action"],
                    color=warning_info["color"],
                    normal_level=normal_warning_info["level"],
                    normal_label=normal_warning_info["label"],
                    normal_action=normal_warning_info["action"],
                    normal_color=normal_warning_info["color"],
                    min_ttc_sec=ttc_result.min_ttc_sec,
                    target_accel_mps2=accel,
                    distance_m=ttc_result.distance_m,
                    collision_time_sec=collision_time_sec,
                )
            )

        self.latest_static_obstacle = None
        if points is not None:
            self.latest_static_obstacle = self.detect_static_obstacle(frame_id, timestamp_sec, points)
            static_warning = self.evaluate_static_obstacles(self.latest_static_obstacle)
            if static_warning is not None:
                warnings.append(static_warning)

        warnings.sort(key=lambda item: item.min_ttc_sec)
        return warnings

    def _trajectory_points_for_ttc(
        self,
        track: TrackedPedestrian,
        trajectory: PredictedTrajectory | None,
    ) -> list[TrajectoryPoint]:
        config = self.config
        horizon_sec = max(config.ttc_horizon_sec, config.level1_ttc_sec, config.prediction_dt_sec)
        step_sec = max(config.prediction_dt_sec, 0.05)
        if trajectory is None or not trajectory.points:
            return self._extrapolated_points(
                start_t_sec=0.0,
                start_x=track.x,
                start_y=track.y,
                velocity_x=track.vx,
                velocity_y=track.vy,
                step_sec=step_sec,
                horizon_sec=horizon_sec,
            )

        points = sorted(
            (point for point in trajectory.points if point.t_sec is None or point.t_sec > 0.0),
            key=lambda point: point.t_sec if point.t_sec is not None else 0.0,
        )
        if not points:
            return self._stationary_points(track)

        last = points[-1]
        last_t_sec = last.t_sec if last.t_sec is not None else len(points) * step_sec
        if last_t_sec >= horizon_sec:
            return points

        velocity_x = track.vx
        velocity_y = track.vy
        if len(points) >= 2:
            prev = points[-2]
            prev_t_sec = prev.t_sec if prev.t_sec is not None else max(0.0, last_t_sec - step_sec)
            dt = last_t_sec - prev_t_sec
            if dt > 1e-6:
                velocity_x = (last.x - prev.x) / dt
                velocity_y = (last.y - prev.y) / dt

        points.extend(
            self._extrapolated_points(
                start_t_sec=last_t_sec,
                start_x=last.x,
                start_y=last.y,
                velocity_x=velocity_x,
                velocity_y=velocity_y,
                step_sec=step_sec,
                horizon_sec=horizon_sec,
            )
        )
        return points

    def _stationary_points(self, track: TrackedPedestrian) -> list[TrajectoryPoint]:
        config = self.config
        horizon_sec = max(config.ttc_horizon_sec, config.level1_ttc_sec, config.prediction_dt_sec)
        step_sec = max(config.prediction_dt_sec, 0.05)
        return self._extrapolated_points(
            start_t_sec=0.0,
            start_x=track.x,
            start_y=track.y,
            velocity_x=0.0,
            velocity_y=0.0,
            step_sec=step_sec,
            horizon_sec=horizon_sec,
        )

    @staticmethod
    def _extrapolated_points(
        start_t_sec: float,
        start_x: float,
        start_y: float,
        velocity_x: float,
        velocity_y: float,
        step_sec: float,
        horizon_sec: float,
    ) -> list[TrajectoryPoint]:
        points: list[TrajectoryPoint] = []
        next_t_sec = min(start_t_sec + step_sec, horizon_sec)
        while next_t_sec <= horizon_sec + 1e-6 and next_t_sec > start_t_sec + 1e-6:
            elapsed_sec = next_t_sec - start_t_sec
            points.append(
                TrajectoryPoint(
                    t_sec=next_t_sec,
                    x=start_x + velocity_x * elapsed_sec,
                    y=start_y + velocity_y * elapsed_sec,
                )
            )
            next_t_sec += step_sec
        return points

    def detect_static_obstacle(
        self,
        frame_id: int,
        timestamp_sec: float,
        points: np.ndarray,
    ) -> StaticObstacleObservation | None:
        config = self.config
        roi_points = self._static_roi_points(points)
        if len(roi_points) < config.static_obstacle_min_points:
            return None

        cluster_points = self._closest_static_cluster(roi_points)
        if len(cluster_points) < config.static_obstacle_min_points:
            return None

        speed_mps = abs(config.ego_speed_mps)
        suppressed_low_speed = False
        obstacle_front_x = float(np.percentile(cluster_points[:, 0], 5.0))
        distance_m = max(0.0, obstacle_front_x - config.vehicle_front_m)
        ttc_sec = distance_m / speed_mps if speed_mps > 1e-6 else float("inf")
        warning_info = self.classify_warning(ttc_sec)
        centroid = np.mean(cluster_points[:, :3], axis=0)

        return StaticObstacleObservation(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            point_count=int(len(cluster_points)),
            level=int(warning_info["level"]),
            label=str(warning_info["label"]),
            suppressed_low_speed=bool(suppressed_low_speed),
            distance_m=distance_m,
            ttc_sec=ttc_sec,
            x_min=float(np.min(cluster_points[:, 0])),
            x_max=float(np.max(cluster_points[:, 0])),
            y_min=float(np.min(cluster_points[:, 1])),
            y_max=float(np.max(cluster_points[:, 1])),
            z_min=float(np.min(cluster_points[:, 2])),
            z_max=float(np.max(cluster_points[:, 2])),
            centroid_x=float(centroid[0]),
            centroid_y=float(centroid[1]),
            centroid_z=float(centroid[2]),
        )

    def evaluate_static_obstacles(
        self,
        observation: StaticObstacleObservation | None,
    ) -> TTCWarning | None:
        if observation is None:
            return None

        warning_info = self.classify_warning(observation.ttc_sec)
        normal_warning_info = self.classify_warning_normal(observation.ttc_sec)
        if warning_info["level"] == 0 and normal_warning_info["level"] == 0:
            return None

        accel = self.target_deceleration(observation.ttc_sec)
        return TTCWarning(
            frame_id=observation.frame_id,
            timestamp_sec=observation.timestamp_sec,
            track_id=-99,
            level=warning_info["level"],
            label="Static_Obstacle",
            action=warning_info["action"],
            color=warning_info["color"],
            normal_level=normal_warning_info["level"],
            normal_label=normal_warning_info["label"],
            normal_action=normal_warning_info["action"],
            normal_color=normal_warning_info["color"],
            min_ttc_sec=observation.ttc_sec,
            target_accel_mps2=accel,
            distance_m=observation.distance_m,
            collision_time_sec=observation.timestamp_sec + observation.ttc_sec,
        )

    def _static_roi_points(self, points: np.ndarray) -> np.ndarray:
        config = self.config
        if points.ndim != 2 or points.shape[1] < 3:
            return np.empty((0, 3), dtype=float)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        in_roi = (
            (x >= config.roi_x_min) &
            (x <= config.roi_x_max) &
            (y >= config.roi_y_min) &
            (y <= config.roi_y_max) &
            (z >= config.roi_z_min) &
            (z <= config.roi_z_max)
        )
        return points[in_roi, :3]

    def _closest_static_cluster(self, roi_points: np.ndarray) -> np.ndarray:
        config = self.config
        if len(roi_points) < config.static_obstacle_min_points:
            return np.empty((0, 3), dtype=float)

        bin_m = max(config.static_cluster_bin_m, 0.05)
        x_bins = np.floor((roi_points[:, 0] - config.roi_x_min) / bin_m).astype(np.int32)
        y_bins = np.floor((roi_points[:, 1] - config.roi_y_min) / bin_m).astype(np.int32)
        cells = np.column_stack((x_bins, y_bins))
        unique_cells, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
        occupied_indices = np.flatnonzero(counts >= max(1, config.static_cluster_min_cell_points))
        if len(occupied_indices) == 0:
            return np.empty((0, 3), dtype=float)

        occupied_cells = {tuple(unique_cells[index]) for index in occupied_indices}
        cell_to_index = {tuple(cell): index for index, cell in enumerate(unique_cells)}
        best_points = np.empty((0, 3), dtype=float)
        best_distance = float("inf")

        while occupied_cells:
            seed = occupied_cells.pop()
            stack = [seed]
            component_cells = [seed]
            while stack:
                cx, cy = stack.pop()
                for nx in range(cx - 1, cx + 2):
                    for ny in range(cy - 1, cy + 2):
                        neighbor = (nx, ny)
                        if neighbor in occupied_cells:
                            occupied_cells.remove(neighbor)
                            stack.append(neighbor)
                            component_cells.append(neighbor)

            component_indices = [cell_to_index[cell] for cell in component_cells]
            component_mask = np.isin(inverse, component_indices)
            component_points = roi_points[component_mask]
            if len(component_points) < config.static_obstacle_min_points:
                continue
            component_distance = float(np.percentile(component_points[:, 0], 5.0))
            if component_distance < best_distance:
                best_distance = component_distance
                best_points = component_points

        return best_points

    def compute_ttc(
        self,
        current_track: TrackedPedestrian,
        points: list[TrajectoryPoint],
    ) -> "_TTCComputation":
        config = self.config
        pedestrian_radius_m = self._pedestrian_collision_radius(current_track)
        collision_margin_m = pedestrian_radius_m + max(config.safety_radius_m, 0.0)
        front_m = config.vehicle_front_m + collision_margin_m
        rear_m = config.vehicle_rear_m + collision_margin_m
        side_m = config.vehicle_side_m + collision_margin_m
        current_physical_distance = self._signed_distance_to_footprint(
            current_track.x,
            current_track.y,
            config.vehicle_front_m,
            config.vehicle_rear_m,
            config.vehicle_side_m,
        )
        if current_physical_distance <= 0.0:
            return _TTCComputation(
                min_ttc_sec=0.0,
                distance_m=0.0,
                collision_time_sec=0.0,
            )

        best_ttc = float("inf")
        prev_t_sec = 0.0
        prev_distance = self._signed_distance_to_future_ego_footprint(
            point_x=current_track.x,
            point_y=current_track.y,
            t_sec=prev_t_sec,
            front_m=front_m,
            rear_m=rear_m,
            side_m=side_m,
        )
        best_distance = max(prev_distance, 0.0)
        best_collision_time = float("inf")

        for index, point in enumerate(points):
            t_sec = point.t_sec if point.t_sec is not None else (index + 1) * config.prediction_dt_sec
            if t_sec <= prev_t_sec:
                continue

            distance = self._signed_distance_to_future_ego_footprint(
                point_x=point.x,
                point_y=point.y,
                t_sec=t_sec,
                front_m=front_m,
                rear_m=rear_m,
                side_m=side_m,
            )

            dt = max(t_sec - prev_t_sec, 1e-6)
            if distance <= 0.0:
                if prev_distance > 0.0:
                    crossing = prev_distance / max(prev_distance - distance, 1e-6)
                    point_ttc = prev_t_sec + min(max(crossing, 0.0), 1.0) * dt
                else:
                    point_ttc = t_sec
                best_ttc = min(best_ttc, point_ttc)
                best_distance = 0.0
                best_collision_time = point_ttc
                break

            clearance = max(distance, 0.0)
            if clearance < best_distance:
                best_distance = clearance
            prev_distance = distance
            prev_t_sec = t_sec

        return _TTCComputation(
            min_ttc_sec=best_ttc,
            distance_m=best_distance,
            collision_time_sec=best_collision_time,
        )

    def _signed_distance_to_future_ego_footprint(
        self,
        point_x: float,
        point_y: float,
        t_sec: float,
        front_m: float,
        rear_m: float,
        side_m: float,
    ) -> float:
        ego_x, ego_y, ego_yaw = self._ego_pose_at(t_sec)
        return self._signed_distance_to_oriented_footprint(
            point_x=point_x,
            point_y=point_y,
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            front_m=front_m,
            rear_m=rear_m,
            side_m=side_m,
        )

    def _ego_pose_at(self, t_sec: float) -> tuple[float, float, float]:
        config = self.config
        speed_mps = config.ego_speed_mps
        road_angle_rad = math.radians(config.ego_steering_deg) / max(config.ego_steer_ratio, 1e-6)
        yaw_rate = speed_mps / max(config.ego_wheelbase_m, 1e-6) * math.tan(road_angle_rad)
        if abs(yaw_rate) < 1e-6:
            return speed_mps * t_sec, 0.0, 0.0

        yaw = yaw_rate * t_sec
        radius = speed_mps / yaw_rate
        return radius * math.sin(yaw), radius * (1.0 - math.cos(yaw)), yaw

    @staticmethod
    def _signed_distance_to_oriented_footprint(
        point_x: float,
        point_y: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        front_m: float,
        rear_m: float,
        side_m: float,
    ) -> float:
        dx = point_x - ego_x
        dy = point_y - ego_y
        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return TTCWarningAdapter._signed_distance_to_footprint(local_x, local_y, front_m, rear_m, side_m)

    @staticmethod
    def _pedestrian_collision_radius(track: TrackedPedestrian) -> float:
        extents = [value for value in (track.dx, track.dy) if value is not None and value > 0.0]
        if not extents:
            return 0.35
        return max(0.25, min(0.60, max(extents) * 0.5))

    @staticmethod
    def _distance_to_footprint(local_x: float, local_y: float, front_m: float, rear_m: float, side_m: float) -> float:
        return max(0.0, TTCWarningAdapter._signed_distance_to_footprint(local_x, local_y, front_m, rear_m, side_m))

    @staticmethod
    def _signed_distance_to_footprint(local_x: float, local_y: float, front_m: float, rear_m: float, side_m: float) -> float:
        dx = max(local_x - front_m, -local_x - rear_m)
        dy = abs(local_y) - side_m
        outside_x = max(dx, 0.0)
        outside_y = max(dy, 0.0)
        outside_distance = math.hypot(outside_x, outside_y)
        if dx <= 0.0 and dy <= 0.0:
            return max(dx, dy)
        return outside_distance

    def target_deceleration(self, ttc_sec: float) -> float:
        config = self.config
        _, level2_ttc_sec, _ = self._active_thresholds()
        speed_mps = abs(config.ego_speed_mps)
        if math.isinf(ttc_sec) or ttc_sec > level2_ttc_sec or speed_mps <= 1e-3:
            return 0.0

        accel_cmd = -speed_mps / max(ttc_sec, 0.1)
        accel_cmd = max(accel_cmd, config.max_decel_mps2)
        return round(accel_cmd, 3)

    def s_curve_deceleration(self, ttc_sec: float) -> float:
        return self.target_deceleration(ttc_sec)

    def classify_warning(self, ttc_sec: float) -> dict:
        return self._classify_warning_with_thresholds(ttc_sec, self._active_thresholds())

    def classify_warning_normal(self, ttc_sec: float) -> dict:
        config = self.config
        return self._classify_warning_with_thresholds(
            ttc_sec,
            (config.level1_ttc_sec, config.level2_ttc_sec, config.level3_ttc_sec),
        )

    @staticmethod
    def _classify_warning_with_thresholds(ttc_sec: float, thresholds: tuple[float, float, float]) -> dict:
        level1_ttc_sec, level2_ttc_sec, level3_ttc_sec = thresholds
        if ttc_sec <= level3_ttc_sec:
            return {
                "level": 3,
                "label": "LEVEL3_AEB",
                "action": "max_decel_candidate",
                "color": "red",
            }
        if ttc_sec <= level2_ttc_sec:
            return {
                "level": 2,
                "label": "LEVEL2_PARTIAL_BRAKE",
                "action": "s_curve_decel_candidate",
                "color": "orange",
            }
        if ttc_sec <= level1_ttc_sec:
            return {
                "level": 1,
                "label": "LEVEL1_FCW",
                "action": "warning_candidate",
                "color": "yellow",
            }
        return {
            "level": 0,
            "label": "SAFE",
            "action": "normal",
            "color": "green",
        }

    def _active_thresholds(self) -> tuple[float, float, float]:
        config = self.config
        return (
            config.level1_ttc_sec,
            config.level2_ttc_sec,
            config.level3_ttc_sec,
        )


@dataclass(frozen=True)
class _TTCComputation:
    min_ttc_sec: float
    distance_m: float
    collision_time_sec: float


def warning_rows(warnings: list[TTCWarning]) -> list[dict]:
    rows = []
    for warning in warnings:
        row = asdict(warning)
        for key in ("min_ttc_sec", "collision_time_sec"):
            if math.isinf(row[key]):
                row[key] = "inf"
        rows.append(row)
    return rows


def write_warnings_json(warnings: list[TTCWarning], json_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(warning_rows(warnings), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
