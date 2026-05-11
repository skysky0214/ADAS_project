from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent.parent))

from app.core.config import PipelineConfig
from app.control.ttc_warning import (
    TTCWarning,
    TTCWarningAdapter,
    TTCWarningConfig,
    write_warnings_json,
)
from app.core.domain_types import (
    FrameInput,
    PredictedTrajectory,
    TrackedPedestrian,
    TrackingFrameResult,
)
from app.core.logging_utils import (
    write_prediction_input_json,
    write_tracking_csv,
    write_tracking_json,
)
from app.bridge.planner_interface import build_planner_snapshot
from app.perception.pedestrian_filter import filter_pedestrians
from app.pipeline import RealTimePedestrianTrackingPipeline
from app.prediction.input_builder import build_prediction_input


def pointcloud2_to_xyzi(msg: PointCloud2) -> np.ndarray:
    field_names = {field.name for field in msg.fields}
    has_intensity = "intensity" in field_names
    requested_fields = ("x", "y", "z", "intensity") if has_intensity else ("x", "y", "z")
    points = pc2.read_points(msg, field_names=requested_fields, skip_nans=True)

    rows = []
    for point in points:
        if has_intensity:
            rows.append((point[0], point[1], point[2], point[3]))
        else:
            rows.append((point[0], point[1], point[2], 0.0))

    if not rows:
        return np.zeros((0, 4), dtype=np.float32)
    return np.asarray(rows, dtype=np.float32)


def msg_timestamp_sec(msg: PointCloud2) -> float:
    stamp = msg.header.stamp
    timestamp_sec = float(stamp.sec) + (float(stamp.nanosec) * 1e-9)
    return timestamp_sec


def write_detection_csv(rows: list[dict], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame",
        "timestamp_sec",
        "class",
        "score",
        "x",
        "y",
        "z",
        "dx",
        "dy",
        "dz",
        "heading",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class DSVTTrackingNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("adas_dsvt_tracking_node")
        self.args = args
        self.frame_count = 0
        self.results: list[TrackingFrameResult] = []
        self.prediction_batches = []
        self.predicted_trajectories: list[dict] = []
        self.planner_snapshots: list[dict] = []
        self.ttc_warnings: list[TTCWarning] = []
        self.detection_rows: list[dict] = []
        self.stop_requested = False
        self.outputs_saved = False

        config = PipelineConfig(
            perception_name="openpcdet_dsvt",
            perception_score_threshold=args.score_threshold,
            perception_device=args.device,
        )
        self.get_logger().info("Loading OpenPCDet DSVT model...")
        self.pipeline = RealTimePedestrianTrackingPipeline(config)
        self.get_logger().info("Model loaded. Waiting for PointCloud2 frames.")
        self.prediction_model = self._build_prediction_model(args, config)
        self.warning_adapter = self._build_warning_adapter(args)
        self.marker_publisher = None
        if not args.no_rviz:
            self.marker_publisher = self.create_publisher(MarkerArray, args.marker_topic, 10)
            self.get_logger().info(f"Publishing RViz markers on {args.marker_topic}")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            args.topic,
            self.on_lidar_points,
            qos,
        )

    def on_lidar_points(self, msg: PointCloud2) -> None:
        if self.args.max_frames is not None and self.frame_count >= self.args.max_frames:
            return

        frame_id = self.frame_count
        timestamp_sec = msg_timestamp_sec(msg)
        points = pointcloud2_to_xyzi(msg)
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            sensor_source=self.args.topic,
            payload={"points": points},
        )

        detections = self.pipeline.detector.infer(frame)
        pedestrian_detections = filter_pedestrians(detections)
        tracks = self.pipeline.tracker.update(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
        )
        result = TrackingFrameResult(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
            tracks=tracks,
        )
        self.results.append(result)
        self.prediction_batches.append(build_prediction_input(result))
        trajectories = []
        if self.prediction_model is not None:
            trajectories = self.prediction_model.predict(result.tracks)
            self.predicted_trajectories.extend(
                _prediction_rows(
                    frame_id=frame.frame_id,
                    timestamp_sec=frame.timestamp_sec,
                    trajectories=trajectories,
                )
            )
        planner_snapshot = build_planner_snapshot(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            tracked_objects=result.tracks,
            predicted_trajectories=trajectories,
        )
        self.planner_snapshots.append(asdict(planner_snapshot))
        warnings = self.warning_adapter.evaluate(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            tracked_objects=result.tracks,
            predicted_trajectories=trajectories,
        )
        self.ttc_warnings.extend(warnings)
        if self.marker_publisher is not None:
            marker_frame = self.args.marker_frame or msg.header.frame_id or "map"
            self.marker_publisher.publish(
                build_tracking_marker_array(
                    frame_id=marker_frame,
                    timestamp=msg.header.stamp,
                    tracks=result.tracks,
                    trajectories=trajectories,
                    warnings=warnings,
                    history_tail=self.args.marker_history_tail,
                )
            )

        for detection in detections:
            item = asdict(detection)
            self.detection_rows.append(
                {
                    "frame": frame_id,
                    "timestamp_sec": timestamp_sec,
                    "class": item.pop("label"),
                    **item,
                }
            )

        if frame_id % self.args.print_every == 0:
            self.get_logger().info(
                f"frame={frame_id} points={len(points)} detections={len(detections)} "
                f"pedestrians={len(pedestrian_detections)} tracks={len(tracks)} "
                f"predicted={len(trajectories)} warnings={len([item for item in warnings if item.level > 0])}"
            )

        self.frame_count += 1
        if self.args.save_every and self.frame_count % self.args.save_every == 0:
            self.save_outputs()

        if self.args.max_frames is not None and self.frame_count >= self.args.max_frames:
            self.get_logger().info("Reached --max-frames. Saving outputs and shutting down.")
            self.save_outputs()
            self.stop_requested = True

    def save_outputs(self) -> None:
        output_dir = self.args.output_dir
        write_tracking_csv(self.results, output_dir / "tracking_results.csv")
        write_tracking_json(self.results, output_dir / "tracking_results.json")
        write_prediction_input_json(self.prediction_batches, output_dir / "prediction_input.json")
        write_detection_csv(self.detection_rows, output_dir / "detections.csv")
        if self.predicted_trajectories:
            write_prediction_csv(self.predicted_trajectories, output_dir / "predicted_trajectories.csv")
            write_prediction_json(self.predicted_trajectories, output_dir / "predicted_trajectories.json")
        write_warnings_json(self.ttc_warnings, output_dir / "ttc_warnings.json")
        write_json(self.planner_snapshots, output_dir / "planner_snapshots.json")
        self.outputs_saved = True
        self.get_logger().info(f"Saved outputs under: {output_dir}")

    def _build_prediction_model(self, args: argparse.Namespace, config: PipelineConfig):
        if args.prediction == "none":
            return None
        if args.prediction == "srlstm":
            self.get_logger().info("Loading SR-LSTM prediction model...")
            from app.prediction.adapters.srlstm_predictor import SRLSTMPredictionModel

            model = SRLSTMPredictionModel(
                checkpoint=config.srlstm_checkpoint,
                sensor_fps=args.prediction_fps,
            )
            self.get_logger().info("SR-LSTM prediction model loaded.")
            return model
        raise ValueError(f"Unknown prediction model: {args.prediction}")

    def _build_warning_adapter(self, args: argparse.Namespace) -> TTCWarningAdapter:
        return TTCWarningAdapter(
            TTCWarningConfig(
                ego_speed_mps=args.ego_speed,
                prediction_dt_sec=1.0 / args.prediction_fps,
                safety_radius_m=args.safety_radius,
            )
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to ROS2 PointCloud2, run OpenPCDet DSVT, and track pedestrians"
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--prediction", choices=["none", "srlstm"], default="none")
    parser.add_argument("--prediction-fps", type=float, default=2.5)
    parser.add_argument("--ego-speed", type=float, default=10.0)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--marker-topic", default="/adas/tracking_markers")
    parser.add_argument("--marker-frame", default=None)
    parser.add_argument("--marker-history-tail", type=int, default=20)
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/live_dsvt_tracking"))
    return parser


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

    return marker_array


def _prediction_rows(
    frame_id: int,
    timestamp_sec: float,
    trajectories: list[PredictedTrajectory],
) -> list[dict]:
    rows = []
    for trajectory in trajectories:
        for point in trajectory.points:
            rows.append(
                {
                    "frame": frame_id,
                    "timestamp_sec": timestamp_sec,
                    "track_id": trajectory.track_id,
                    "t_sec": point.t_sec,
                    "x": point.x,
                    "y": point.y,
                    "confidence": trajectory.confidence,
                    "model": trajectory.model_name,
                }
            )
    return rows


def write_prediction_csv(rows: list[dict], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame",
        "timestamp_sec",
        "track_id",
        "t_sec",
        "x",
        "y",
        "confidence",
        "model",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_prediction_json(rows: list[dict], json_path: Path) -> None:
    write_json(rows, json_path)


def write_json(rows: list[dict], json_path: Path) -> None:
    import json

    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = DSVTTrackingNode(args)

    def handle_signal(signum, frame):
        node.get_logger().info("Signal received. Saving outputs before shutdown.")
        node.save_outputs()
        node.stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok() and not node.outputs_saved:
            node.save_outputs()
            rclpy.shutdown()
        elif rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
