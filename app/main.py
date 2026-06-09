from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

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
    TrackingFrameResult,
)
from app.io.artifacts import (
    prediction_rows,
    write_detection_csv,
    write_json,
    write_latency_csv,
    write_prediction_csv,
    write_prediction_json,
)
from app.io.pointcloud import msg_timestamp_sec, pointcloud2_to_xyzi
from app.core.logging_utils import (
    write_prediction_input_json,
    write_tracking_csv,
    write_tracking_json,
)
from app.bridge.planner_interface import build_planner_snapshot
from app.perception.pedestrian_filter import filter_pedestrians
from app.pipeline import RealTimePedestrianTrackingPipeline
from app.prediction.input_builder import build_prediction_input
from app.visualization.rviz_markers import build_tracking_marker_array


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
        self.latency_rows: list[dict] = []
        self.first_msg_timestamp_sec: float | None = None
        self.first_receive_wall_sec: float | None = None
        self.stop_requested = False
        self.outputs_saved = False

        config = PipelineConfig(
            perception_name=f"openpcdet_{args.perception}",
            perception_score_threshold=args.score_threshold,
            perception_device=args.device,
        )
        self.get_logger().info(f"Loading OpenPCDet {args.perception} model...")
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

        callback_start = time.perf_counter()
        receive_wall_sec = time.time()
        frame_id = self.frame_count
        timestamp_sec = msg_timestamp_sec(msg)
        if self.first_msg_timestamp_sec is None:
            self.first_msg_timestamp_sec = timestamp_sec
            self.first_receive_wall_sec = receive_wall_sec

        stage_start = time.perf_counter()
        points = pointcloud2_to_xyzi(msg)
        pointcloud_ms = _elapsed_ms(stage_start)
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            sensor_source=self.args.topic,
            payload={"points": points},
        )

        stage_start = time.perf_counter()
        detections = self.pipeline.detector.infer(frame)
        perception_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        pedestrian_detections = filter_pedestrians(detections)
        filter_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        tracks = self.pipeline.tracker.update(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
        )
        tracking_ms = _elapsed_ms(stage_start)
        result = TrackingFrameResult(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
            tracks=tracks,
        )
        self.results.append(result)
        self.prediction_batches.append(build_prediction_input(result))
        trajectories = []
        prediction_ms = 0.0
        if self.prediction_model is not None:
            stage_start = time.perf_counter()
            trajectories = self.prediction_model.predict(result.tracks)
            prediction_ms = _elapsed_ms(stage_start)
            self.predicted_trajectories.extend(
                prediction_rows(
                    frame_id=frame.frame_id,
                    timestamp_sec=frame.timestamp_sec,
                    trajectories=trajectories,
                )
            )
        stage_start = time.perf_counter()
        planner_snapshot = build_planner_snapshot(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            tracked_objects=result.tracks,
            predicted_trajectories=trajectories,
        )
        self.planner_snapshots.append(asdict(planner_snapshot))
        planner_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        warnings = self.warning_adapter.evaluate(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            tracked_objects=result.tracks,
            predicted_trajectories=trajectories,
            points=points,
        )
        self.ttc_warnings.extend(warnings)
        warning_ms = _elapsed_ms(stage_start)
        marker_ms = 0.0
        if self.marker_publisher is not None:
            stage_start = time.perf_counter()
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
            marker_ms = _elapsed_ms(stage_start)

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
            total_ms = _elapsed_ms(callback_start)
            playback_lag_ms = self._playback_lag_ms(timestamp_sec, receive_wall_sec)
            self.get_logger().info(
                f"frame={frame_id} points={len(points)} detections={len(detections)} "
                f"pedestrians={len(pedestrian_detections)} tracks={len(tracks)} "
                f"predicted={len(trajectories)} warnings={len([item for item in warnings if item.level > 0])} "
                f"latency_total={total_ms:.1f}ms perception={perception_ms:.1f}ms "
                f"prediction={prediction_ms:.1f}ms replay_lag={playback_lag_ms:.1f}ms"
            )

        self.latency_rows.append(
            {
                "frame": frame_id,
                "msg_timestamp_sec": timestamp_sec,
                "receive_wall_sec": receive_wall_sec,
                "playback_lag_ms": self._playback_lag_ms(timestamp_sec, receive_wall_sec),
                "total_callback_ms": _elapsed_ms(callback_start),
                "pointcloud_ms": pointcloud_ms,
                "perception_ms": perception_ms,
                "filter_ms": filter_ms,
                "tracking_ms": tracking_ms,
                "prediction_ms": prediction_ms,
                "planner_ms": planner_ms,
                "warning_ms": warning_ms,
                "marker_ms": marker_ms,
                "points": len(points),
                "detections": len(detections),
                "pedestrians": len(pedestrian_detections),
                "tracks": len(tracks),
                "predicted": len(trajectories),
                "active_warnings": len([item for item in warnings if item.level > 0]),
            }
        )

        self.frame_count += 1
        if self.args.save_every and self.frame_count % self.args.save_every == 0:
            self.save_outputs()

        if self.args.max_frames is not None and self.frame_count >= self.args.max_frames:
            self.get_logger().info("Reached --max-frames. Saving outputs and shutting down.")
            self.save_outputs()
            self.stop_requested = True

    def save_outputs(self) -> None:
        if self.args.output_dir is None:
            self.outputs_saved = True
            self.get_logger().info("No --output-dir set. Skipping artifact export.")
            return

        output_dir = self.args.output_dir
        write_tracking_csv(self.results, output_dir / "tracking_results.csv")
        write_tracking_json(self.results, output_dir / "tracking_results.json")
        write_prediction_input_json(self.prediction_batches, output_dir / "prediction_input.json")
        write_detection_csv(self.detection_rows, output_dir / "detections.csv")
        write_latency_csv(self.latency_rows, output_dir / "latency.csv")
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
                roi_x_min=args.roi_x_min,
                roi_x_max=args.roi_x_max,
                roi_y_min=args.roi_y_min,
                roi_y_max=args.roi_y_max,
                roi_z_min=args.roi_z_min,
                roi_z_max=args.roi_z_max,
                static_obstacle_min_points=args.static_min_points,
            )
        )

    def _playback_lag_ms(self, timestamp_sec: float, receive_wall_sec: float) -> float:
        if self.first_msg_timestamp_sec is None or self.first_receive_wall_sec is None:
            return 0.0
        msg_elapsed = timestamp_sec - self.first_msg_timestamp_sec
        wall_elapsed = receive_wall_sec - self.first_receive_wall_sec
        expected_wall_elapsed = msg_elapsed / max(self.args.latency_playback_rate, 1e-6)
        return (wall_elapsed - expected_wall_elapsed) * 1000.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to ROS2 PointCloud2, run OpenPCDet perception, and track pedestrians"
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--perception", choices=["dsvt", "pointpillar"], default="pointpillar")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--prediction", choices=["none", "srlstm"], default="none")
    parser.add_argument("--prediction-fps", type=float, default=2.5)
    parser.add_argument(
        "--latency-playback-rate",
        type=float,
        default=1.0,
        help="rosbag play rate used to estimate replay lag from message stamps",
    )
    parser.add_argument("--ego-speed", type=float, default=10.0)
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument("--roi-x-min", type=float, default=1.0)
    parser.add_argument("--roi-x-max", type=float, default=15.0)
    parser.add_argument("--roi-y-min", type=float, default=-1.0)
    parser.add_argument("--roi-y-max", type=float, default=1.0)
    parser.add_argument("--roi-z-min", type=float, default=-1.4)
    parser.add_argument("--roi-z-max", type=float, default=1.0)
    parser.add_argument("--static-min-points", type=int, default=15)
    parser.add_argument("--marker-topic", default="/adas/tracking_markers")
    parser.add_argument("--marker-frame", default=None)
    parser.add_argument("--marker-history-tail", type=int, default=20)
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def _elapsed_ms(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def main() -> None:
    args = build_parser().parse_args()
    rclpy.init()
    node = DSVTTrackingNode(args)

    def handle_signal(signum, frame):
        node.get_logger().info("Signal received. Shutting down.")
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
