from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parents[2]))

from app.core.config import (
    DEFAULT_DSVT_CFG_FILE,
    DEFAULT_DSVT_CHECKPOINT,
    DEFAULT_OPENPCDET_ROOT,
    DEFAULT_POINTPILLAR_CFG_FILE,
    DEFAULT_POINTPILLAR_CHECKPOINT,
)
from app.core.domain_types import FrameInput, TrackedPedestrian
from app.perception.adapters.openpcdet_dsvt import OpenPCDetDSVTPerceptionModel
from app.perception.adapters.openpcdet_pointpillar import OpenPCDetPointPillarPerceptionModel
from app.perception.pedestrian_filter import PedestrianPointSpreadFilter, filter_pedestrians
from app.tracking.pedestrian_tracker import PedestrianTracker


PEDESTRIAN_TRACK_COLOR = (0.0, 0.6, 1.0)
DETECTION_CLASS_COLORS: dict[str, tuple[float, float, float]] = {
    "Pedestrian": (0.0, 0.6, 1.0),
    "Cyclist": (0.0, 0.85, 0.2),
    "Vehicle": (1.0, 0.2, 0.2),
    "Car": (1.0, 0.2, 0.2),
}
DEFAULT_DETECTION_COLOR = (1.0, 0.75, 0.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run OpenPCDet detection + pedestrian tracking and visualize with Open3D"
    )
    parser.add_argument("data_path", type=Path, help="Point cloud file or directory of .npy/.bin frames")
    parser.add_argument("--perception", choices=["dsvt", "pointpillar"], default="pointpillar")
    parser.add_argument("--openpcdet-root", type=Path, default=DEFAULT_OPENPCDET_ROOT)
    parser.add_argument("--dsvt-cfg-file", type=Path, default=DEFAULT_DSVT_CFG_FILE)
    parser.add_argument("--dsvt-checkpoint", type=Path, default=DEFAULT_DSVT_CHECKPOINT)
    parser.add_argument("--pointpillar-cfg-file", type=Path, default=DEFAULT_POINTPILLAR_CFG_FILE)
    parser.add_argument("--pointpillar-checkpoint", type=Path, default=DEFAULT_POINTPILLAR_CHECKPOINT)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument(
        "--pedestrian-min-point-max-distance",
        type=float,
        default=None,
        help="Drop Pedestrian detections whose max point-to-point distance is smaller than this value in meters",
    )
    parser.add_argument(
        "--pedestrian-max-point-max-distance",
        type=float,
        default=None,
        help="Drop Pedestrian detections whose max point-to-point distance is larger than this value in meters",
    )
    parser.add_argument("--ext", choices=[".npy", ".bin"], default=".npy")
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--history-tail", type=int, default=10)
    parser.add_argument("--point-size", type=float, default=2.0)
    parser.add_argument("--track-radius", type=float, default=0.12)
    parser.add_argument("--track-marker-radius", type=float, default=0.18)
    parser.add_argument("--detection-radius", type=float, default=0.28)
    parser.add_argument("--match-distance", type=float, default=1.2)
    parser.add_argument("--reconnect-distance", type=float, default=2.4)
    parser.add_argument("--max-missed", type=int, default=5)
    parser.add_argument("--history-size", type=int, default=10)
    return parser


def build_detector(args: argparse.Namespace):
    if args.perception == "dsvt":
        return OpenPCDetDSVTPerceptionModel(
            openpcdet_root=args.openpcdet_root,
            cfg_file=args.dsvt_cfg_file,
            checkpoint=args.dsvt_checkpoint,
            score_threshold=args.score_threshold,
            device=args.device,
        )
    return OpenPCDetPointPillarPerceptionModel(
        openpcdet_root=args.openpcdet_root,
        cfg_file=args.pointpillar_cfg_file,
        checkpoint=args.pointpillar_checkpoint,
        score_threshold=args.score_threshold,
        device=args.device,
    )


def collect_frame_paths(data_path: Path, ext: str) -> list[Path]:
    if data_path.is_file():
        return [data_path]
    paths = sorted(path for path in data_path.glob(f"*{ext}") if path.is_file())
    if not paths:
        raise FileNotFoundError(f"No {ext} files found in {data_path}")
    return paths


def load_points(point_path: Path) -> np.ndarray:
    if point_path.suffix == ".npy":
        points = np.load(point_path).astype(np.float32)
    elif point_path.suffix == ".bin":
        points = np.fromfile(point_path, dtype=np.float32).reshape(-1, 4)
    else:
        raise ValueError(f"Unsupported point cloud file extension: {point_path.suffix}")
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Point cloud must have shape [N, >=3], got {points.shape}")
    if points.shape[1] == 3:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.concatenate([points, intensity], axis=1)
    return np.ascontiguousarray(points[:, :4], dtype=np.float32)


def create_track_marker(
    position: np.ndarray,
    color: tuple[float, float, float],
    radius: float,
) -> o3d.geometry.TriangleMesh:
    mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(color)
    mesh.translate(position)
    return mesh


def detection_color(label: str) -> tuple[float, float, float]:
    return DETECTION_CLASS_COLORS.get(label, DEFAULT_DETECTION_COLOR)


def create_cylinder_between_points(
    start_pt: np.ndarray,
    end_pt: np.ndarray,
    color: tuple[float, float, float],
    radius: float,
) -> o3d.geometry.TriangleMesh | None:
    line_vec = end_pt - start_pt
    line_length = np.linalg.norm(line_vec)
    if line_length < 1e-6:
        return None

    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=line_length)
    cylinder.compute_vertex_normals()
    cylinder.paint_uniform_color(color)

    z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    dir_vec = line_vec / line_length
    axis = np.cross(z_axis, dir_vec)
    axis_norm = np.linalg.norm(axis)
    if axis_norm > 1e-6:
        axis = axis / axis_norm
        angle = np.arccos(np.clip(np.dot(z_axis, dir_vec), -1.0, 1.0))
        rotation = cylinder.get_rotation_matrix_from_axis_angle(axis * angle)
        cylinder.rotate(rotation, center=(0.0, 0.0, 0.0))
    elif np.dot(z_axis, dir_vec) < 0:
        rotation = cylinder.get_rotation_matrix_from_axis_angle(np.array([1.0, 0.0, 0.0]) * np.pi)
        cylinder.rotate(rotation, center=(0.0, 0.0, 0.0))

    cylinder.translate((start_pt + end_pt) / 2.0)
    return cylinder


def build_history_geometries(
    track: TrackedPedestrian,
    color: tuple[float, float, float],
    tail: int,
    track_radius: float,
    marker_radius: float,
) -> list[o3d.geometry.Geometry]:
    geometries: list[o3d.geometry.Geometry] = []
    history = track.history[-tail:]
    if len(history) < 2:
        geometries.append(
            create_track_marker(
                position=np.array([track.x, track.y, track.z], dtype=np.float64),
                color=color,
                radius=marker_radius,
            )
        )
        return geometries

    hist_points = np.array([[point.x, point.y, track.z] for point in history], dtype=np.float64)
    for idx in range(len(hist_points) - 1):
        cylinder = create_cylinder_between_points(
            hist_points[idx],
            hist_points[idx + 1],
            color=color,
            radius=track_radius,
        )
        if cylinder is not None:
            geometries.append(cylinder)

    geometries.append(
        create_track_marker(
            position=hist_points[-1],
            color=color,
            radius=marker_radius,
        )
    )
    return geometries


def build_detection_geometries(
    detections: list,
    detection_radius: float,
) -> list[o3d.geometry.Geometry]:
    geometries: list[o3d.geometry.Geometry] = []
    for detection in detections:
        geometries.append(
            create_track_marker(
                position=np.array([detection.x, detection.y, detection.z], dtype=np.float64),
                color=detection_color(detection.label),
                radius=detection_radius,
            )
        )
    return geometries


def main() -> None:
    args = build_parser().parse_args()
    frame_paths = collect_frame_paths(args.data_path, args.ext)
    if args.max_frames is not None:
        frame_paths = frame_paths[: args.max_frames]

    detector = build_detector(args)
    pedestrian_point_spread_filter = PedestrianPointSpreadFilter(
        min_point_max_distance_m=args.pedestrian_min_point_max_distance,
        max_point_max_distance_m=args.pedestrian_max_point_max_distance,
    )
    tracker = PedestrianTracker(
        match_distance=args.match_distance,
        reconnect_distance=args.reconnect_distance,
        max_missed=args.max_missed,
        history_size=args.history_size,
    )

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="OpenPCDet Tracking Open3D")
    vis.get_render_option().point_size = args.point_size
    vis.get_render_option().background_color = np.zeros(3)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(axis)

    point_cloud = o3d.geometry.PointCloud()
    vis.add_geometry(point_cloud)
    dynamic_geometries: list[o3d.geometry.Geometry] = []

    for frame_id, point_path in enumerate(frame_paths):
        points = load_points(point_path)
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=frame_id / max(args.fps, 1e-6),
            sensor_source="point_cloud_file",
            payload={"points": points},
        )
        detections = detector.infer(frame)
        pedestrian_detections = filter_pedestrians(
            detections,
            point_spread_filter=pedestrian_point_spread_filter,
        )
        tracks = tracker.update(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
        )

        point_cloud.points = o3d.utility.Vector3dVector(points[:, :3])
        point_cloud.colors = o3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
        vis.update_geometry(point_cloud)

        for geometry in dynamic_geometries:
            vis.remove_geometry(geometry, reset_bounding_box=False)
        dynamic_geometries.clear()

        for geometry in build_detection_geometries(
            detections=detections,
            detection_radius=args.detection_radius,
        ):
            vis.add_geometry(geometry, reset_bounding_box=False)
            dynamic_geometries.append(geometry)

        for track in tracks:
            for geometry in build_history_geometries(
                track,
                color=PEDESTRIAN_TRACK_COLOR,
                tail=args.history_tail,
                track_radius=args.track_radius,
                marker_radius=args.track_marker_radius,
            ):
                vis.add_geometry(geometry, reset_bounding_box=False)
                dynamic_geometries.append(geometry)

        print(
            f"frame={frame_id} file={point_path.name} detections={len(detections)} "
            f"pedestrians={len(pedestrian_detections)} tracks={len(tracks)}"
        )

        if frame_id == 0:
            vis.reset_view_point(True)
        vis.poll_events()
        vis.update_renderer()
        time.sleep(1.0 / max(args.fps, 1e-6))

    print("Finished replay. Close the Open3D window to exit.")
    while True:
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.05)


if __name__ == "__main__":
    main()
