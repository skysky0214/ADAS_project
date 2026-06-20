from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from app.core.domain_types import PredictedTrajectory, TrackedPedestrian, TrajectoryPoint


@dataclass(frozen=True)
class TTCWarningConfig:
    level1_ttc_sec: float = 1.70
    level2_ttc_sec: float = 1.50
    level3_ttc_sec: float = 0.80
    prediction_dt_sec: float = 0.4
    safety_radius_m: float = 1.0
    ego_speed_mps: float = 10.0
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
    roi_z_min: float = -1.4
    roi_z_max: float = 1.0
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
        track_by_id = {track.track_id: track for track in tracked_objects}
        warnings = []
        prediction_ttc_enabled = abs(self.config.ego_speed_mps) > self.config.low_speed_suppress_mps
        if prediction_ttc_enabled:
            for trajectory in predicted_trajectories:
                track = track_by_id.get(trajectory.track_id)
                if track is None:
                    continue

                ttc_result = self.compute_ttc(track, trajectory.points)
                warning_info = self.classify_warning(ttc_result.min_ttc_sec)
                accel = self.s_curve_deceleration(ttc_result.min_ttc_sec)
                warnings.append(
                    TTCWarning(
                        frame_id=frame_id,
                        timestamp_sec=timestamp_sec,
                        track_id=trajectory.track_id,
                        level=warning_info["level"],
                        label=warning_info["label"],
                        action=warning_info["action"],
                        color=warning_info["color"],
                        min_ttc_sec=ttc_result.min_ttc_sec,
                        target_accel_mps2=accel,
                        distance_m=ttc_result.distance_m,
                        collision_time_sec=ttc_result.collision_time_sec,
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
        suppressed_low_speed = speed_mps <= config.perception_low_speed_suppress_mps
        distance_m = float(np.percentile(cluster_points[:, 0], 5.0))
        ttc_sec = distance_m / speed_mps if speed_mps > 1e-6 else float("inf")
        warning_info = self.classify_warning(ttc_sec) if not suppressed_low_speed else self.classify_warning(float("inf"))
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
        if observation is None or observation.suppressed_low_speed or observation.level == 0:
            return None

        warning_info = self.classify_warning(observation.ttc_sec)
        accel = self.s_curve_deceleration(observation.ttc_sec)
        return TTCWarning(
            frame_id=observation.frame_id,
            timestamp_sec=observation.timestamp_sec,
            track_id=-99,
            level=warning_info["level"],
            label="Static_Obstacle",
            action=warning_info["action"],
            color=warning_info["color"],
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
        front_m = config.vehicle_front_m + config.safety_radius_m
        rear_m = config.vehicle_rear_m + config.safety_radius_m
        side_m = config.vehicle_side_m + config.safety_radius_m
        prev_distance = self._distance_to_footprint(
            current_track.x,
            current_track.y,
            front_m,
            rear_m,
            side_m,
        )
        if prev_distance <= 0.0:
            return _TTCComputation(
                min_ttc_sec=0.0,
                distance_m=0.0,
                collision_time_sec=0.0,
            )

        best_ttc = float("inf")
        best_distance = prev_distance
        best_collision_time = float("inf")

        for index, point in enumerate(points):
            t_sec = point.t_sec if point.t_sec is not None else (index + 1) * config.prediction_dt_sec
            distance = self._distance_to_footprint(
                point.x - config.ego_speed_mps * t_sec,
                point.y,
                front_m,
                rear_m,
                side_m,
            )

            prev_t_sec = 0.0 if index == 0 else points[index - 1].t_sec
            dt = max(t_sec - prev_t_sec, 1e-6)
            approach_speed = (prev_distance - distance) / dt
            speed_ttc = distance / approach_speed if approach_speed > 0.0 and distance > 0.0 else float("inf")

            direct_ttc = t_sec if distance <= 0.0 else float("inf")

            point_ttc = min(speed_ttc, direct_ttc)
            if point_ttc < best_ttc:
                best_ttc = point_ttc
                best_distance = distance
                best_collision_time = t_sec

            prev_distance = distance

        return _TTCComputation(
            min_ttc_sec=best_ttc,
            distance_m=best_distance,
            collision_time_sec=best_collision_time,
        )

    @staticmethod
    def _distance_to_footprint(local_x: float, local_y: float, front_m: float, rear_m: float, side_m: float) -> float:
        dx = max(local_x - front_m, -local_x - rear_m, 0.0)
        dy = max(abs(local_y) - side_m, 0.0)
        return math.hypot(dx, dy)

    def s_curve_deceleration(self, ttc_sec: float) -> float:
        config = self.config
        level1_ttc_sec, _, level3_ttc_sec = self._active_thresholds()
        if math.isinf(ttc_sec) or ttc_sec > level1_ttc_sec:
            return 0.0

        ttc_mid = (level1_ttc_sec + level3_ttc_sec) / 2.0
        k = 5.0
        sigmoid = 1.0 / (1.0 + math.exp(k * (ttc_sec - ttc_mid)))
        accel_cmd = config.max_decel_mps2 * sigmoid
        accel_cmd = max(accel_cmd, config.max_decel_mps2)
        return round(accel_cmd, 3)

    def classify_warning(self, ttc_sec: float) -> dict:
        level1_ttc_sec, level2_ttc_sec, level3_ttc_sec = self._active_thresholds()
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
        scale = config.brake_ttc_scale if config.driver_brake_pressed and not config.driver_accelerator_pressed else 1.0
        scale = max(0.05, min(1.0, scale))
        return (
            config.level1_ttc_sec * scale,
            config.level2_ttc_sec * scale,
            config.level3_ttc_sec * scale,
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
