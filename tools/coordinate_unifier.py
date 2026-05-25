from __future__ import annotations

from dataclasses import dataclass, replace
import math

import numpy as np

from adas.detection.types import Detection3D


class CoordinateError(ValueError):
    """Raised when a coordinate transform input has an invalid shape."""


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


@dataclass(slots=True)
class VehiclePose2D:
    """Vehicle pose in a common world/map frame."""

    x: float
    y: float
    yaw: float

    @classmethod
    def from_degrees(
        cls,
        x: float,
        y: float,
        yaw_deg: float,
    ) -> VehiclePose2D:
        return cls(x=x, y=y, yaw=math.radians(yaw_deg))

    def vehicle_to_world(self) -> RigidTransform2D:
        return RigidTransform2D(self.x, self.y, self.yaw)


class RigidTransform2D:
    """SE(2) transform for x/y/yaw data.

    The transform maps coordinates from a source frame into a target frame:
    target_point = R @ source_point + t.
    """

    __slots__ = ("x", "y", "yaw")

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)

    @classmethod
    def identity(cls) -> RigidTransform2D:
        return cls()

    @classmethod
    def from_degrees(
        cls,
        x: float = 0.0,
        y: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> RigidTransform2D:
        return cls(x=x, y=y, yaw=math.radians(yaw_deg))

    @property
    def rotation_matrix(self) -> np.ndarray:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return np.array(
            [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]],
            dtype=np.float64,
        )

    def inverse(self) -> RigidTransform2D:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        inv_x = (-cos_yaw * self.x) - (sin_yaw * self.y)
        inv_y = (sin_yaw * self.x) - (cos_yaw * self.y)
        return RigidTransform2D(inv_x, inv_y, -self.yaw)

    def compose(self, other: RigidTransform2D) -> RigidTransform2D:
        rotated = self.transform_xy([[other.x, other.y]])[0]
        return RigidTransform2D(
            x=float(rotated[0]),
            y=float(rotated[1]),
            yaw=normalize_yaw(self.yaw + other.yaw),
        )

    def transform_xy(self, xy: np.ndarray | list[list[float]]) -> np.ndarray:
        points = np.asarray(xy, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise CoordinateError("xy must have shape (N, 2)")
        return (points @ self.rotation_matrix.T) + np.array([self.x, self.y])

    def transform_detection(self, detection: Detection3D) -> Detection3D:
        xy = self.transform_xy([[detection.center_x, detection.center_y]])[0]
        velocity_x = detection.velocity_x
        velocity_y = detection.velocity_y
        if velocity_x is not None and velocity_y is not None:
            velocity = np.array([[velocity_x, velocity_y]], dtype=np.float64)
            rotated_velocity = velocity @ self.rotation_matrix.T
            velocity_x = float(rotated_velocity[0, 0])
            velocity_y = float(rotated_velocity[0, 1])

        return replace(
            detection,
            center_x=float(xy[0]),
            center_y=float(xy[1]),
            yaw=normalize_yaw(detection.yaw + self.yaw),
            velocity_x=velocity_x,
            velocity_y=velocity_y,
        )

    def transform_detections(self, detections: list[Detection3D]) -> list[Detection3D]:
        return [self.transform_detection(detection) for detection in detections]


class RigidTransform3D:
    """Yaw-only 3D rigid transform for lidar-to-vehicle projection."""

    __slots__ = ("x", "y", "z", "yaw")

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.yaw = float(yaw)

    @classmethod
    def identity(cls) -> RigidTransform3D:
        return cls()

    @classmethod
    def from_degrees(
        cls,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        yaw_deg: float = 0.0,
    ) -> RigidTransform3D:
        return cls(x=x, y=y, z=z, yaw=math.radians(yaw_deg))

    def as_2d(self) -> RigidTransform2D:
        return RigidTransform2D(self.x, self.y, self.yaw)

    @property
    def rotation_matrix(self) -> np.ndarray:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return np.array(
            [
                [cos_yaw, -sin_yaw, 0.0],
                [sin_yaw, cos_yaw, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def transform_xyz(self, xyz: np.ndarray | list[list[float]]) -> np.ndarray:
        points = np.asarray(xyz, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise CoordinateError("xyz must have shape (N, 3)")
        translation = np.array([self.x, self.y, self.z], dtype=np.float64)
        return (points @ self.rotation_matrix.T) + translation

    def transform_detection(self, detection: Detection3D) -> Detection3D:
        xyz = self.transform_xyz(
            [[detection.center_x, detection.center_y, detection.center_z]]
        )[0]
        transformed_2d = self.as_2d().transform_detection(detection)
        return replace(
            transformed_2d,
            center_x=float(xyz[0]),
            center_y=float(xyz[1]),
            center_z=float(xyz[2]),
        )

    def transform_detections(self, detections: list[Detection3D]) -> list[Detection3D]:
        return [self.transform_detection(detection) for detection in detections]


@dataclass(slots=True)
class CoordinateCalibration:
    """Physical sensor calibration values.

    Keep lidar_to_vehicle as None until the real sensor extrinsic is known.
    """

    lidar_to_vehicle: RigidTransform3D | None = None

    def require_lidar_to_vehicle(self) -> RigidTransform3D:
        if self.lidar_to_vehicle is None:
            raise CoordinateError(
                "lidar_to_vehicle calibration is not set. "
                "Fill it with the lidar position/orientation in the vehicle frame."
            )
        return self.lidar_to_vehicle


@dataclass(slots=True)
class CoordinateFrameState:
    """Coordinate-normalized detections for one timestamp."""

    timestamp_ns: int
    vehicle_pose: VehiclePose2D | None
    detections_vehicle: list[Detection3D]


class CoordinateUnifier:
    """End-to-end coordinate correction before trajectory prediction.

    Output detections are always in the current vehicle frame. If a previous
    frame and ego poses are available, velocity is computed after compensating
    vehicle motion between frames.
    """

    def __init__(self, calibration: CoordinateCalibration | None = None) -> None:
        self.calibration = calibration or CoordinateCalibration()
        self.previous_state: CoordinateFrameState | None = None

    def reset(self) -> None:
        self.previous_state = None

    def transform_lidar_points_to_vehicle(self, xyz_lidar: np.ndarray) -> np.ndarray:
        return self.calibration.require_lidar_to_vehicle().transform_xyz(xyz_lidar)

    def transform_lidar_detections_to_vehicle(
        self,
        detections_lidar: list[Detection3D],
    ) -> list[Detection3D]:
        return self.calibration.require_lidar_to_vehicle().transform_detections(
            detections_lidar
        )

    def process_frame(
        self,
        detections_lidar: list[Detection3D],
        timestamp_ns: int,
        vehicle_pose: VehiclePose2D | None = None,
    ) -> CoordinateFrameState:
        detections_vehicle = self.transform_lidar_detections_to_vehicle(
            detections_lidar
        )

        if self.previous_state is not None:
            dt_sec = (timestamp_ns - self.previous_state.timestamp_ns) / 1e9
            if dt_sec > 0.0:
                previous_to_current = self._previous_to_current_vehicle(
                    previous_pose=self.previous_state.vehicle_pose,
                    current_pose=vehicle_pose,
                )
                detections_vehicle = attach_velocity_from_previous_frame(
                    current=detections_vehicle,
                    previous=self.previous_state.detections_vehicle,
                    dt_sec=dt_sec,
                    previous_to_current_vehicle=previous_to_current,
                )

        state = CoordinateFrameState(
            timestamp_ns=timestamp_ns,
            vehicle_pose=vehicle_pose,
            detections_vehicle=detections_vehicle,
        )
        self.previous_state = state
        return state

    def _previous_to_current_vehicle(
        self,
        previous_pose: VehiclePose2D | None,
        current_pose: VehiclePose2D | None,
    ) -> RigidTransform2D:
        if previous_pose is None or current_pose is None:
            return RigidTransform2D.identity()

        world_to_current = current_pose.vehicle_to_world().inverse()
        previous_to_world = previous_pose.vehicle_to_world()
        return world_to_current.compose(previous_to_world)


def attach_velocity_from_previous_frame(
    current: list[Detection3D],
    previous: list[Detection3D],
    dt_sec: float,
    previous_to_current_vehicle: RigidTransform2D | None = None,
) -> list[Detection3D]:
    """Attach ego-motion-compensated velocity to current detections.

    Inputs are expected in the current vehicle frame, except previous detections
    which are first transformed by previous_to_current_vehicle when provided.
    """

    if dt_sec <= 0.0:
        raise CoordinateError("dt_sec must be positive")

    previous_transform = previous_to_current_vehicle or RigidTransform2D.identity()
    previous_by_id = {
        detection.track_id: previous_transform.transform_detection(detection)
        for detection in previous
        if detection.track_id is not None
    }

    updated: list[Detection3D] = []
    for detection in current:
        previous_detection = previous_by_id.get(detection.track_id)
        if previous_detection is None:
            updated.append(detection)
            continue

        velocity_x = (detection.center_x - previous_detection.center_x) / dt_sec
        velocity_y = (detection.center_y - previous_detection.center_y) / dt_sec
        updated.append(
            replace(
                detection,
                velocity_x=float(velocity_x),
                velocity_y=float(velocity_y),
            )
        )
    return updated
