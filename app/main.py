from __future__ import annotations

import argparse
import signal
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent.parent))

from app.core.config import PipelineConfig
from app.ego_motion import EgoMotionDelta, PandaEgoMotionReader
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
        self.ego_reader = self._build_ego_reader(args)
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
        raw_points = pointcloud2_to_xyzi(msg)
        pointcloud_convert_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        points = _filter_points_by_roi(raw_points, self.args.perception_roi_m)
        pointcloud_roi_ms = _elapsed_ms(stage_start)
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            sensor_source=self.args.topic,
            payload={"points": points},
        )

        stage_start = time.perf_counter()
        raw_detections = self.pipeline.detector.infer(frame)
        detector_infer_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        detections = _filter_detections_by_roi(raw_detections, self.args.perception_roi_m)
        detection_roi_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        pedestrian_detections = filter_pedestrians(detections)
        filter_ms = _elapsed_ms(stage_start)

        stage_start = time.perf_counter()
        ego_delta = self._pop_ego_motion_delta()
        ego_pop_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        if ego_delta.valid:
            self.warning_adapter.config = replace(
                self.warning_adapter.config,
                ego_speed_mps=ego_delta.speed_mps,
                driver_brake_pressed=ego_delta.brake_pressed,
            )
        ego_config_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        if ego_delta.valid or ego_delta.reset:
            self.pipeline.tracker.apply_ego_motion(
                dx_m=ego_delta.dx_m,
                dy_m=ego_delta.dy_m,
                dyaw_rad=ego_delta.dyaw_rad,
                reset=ego_delta.reset and self.args.ego_reset_tracks_on_stop,
            )
        ego_apply_ms = _elapsed_ms(stage_start)

        stage_start = time.perf_counter()
        tracks = self.pipeline.tracker.update(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
        )
        tracking_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        result = TrackingFrameResult(
            frame_id=frame.frame_id,
            timestamp_sec=frame.timestamp_sec,
            detections=pedestrian_detections,
            tracks=tracks,
        )
        self.results.append(result)
        self.prediction_batches.append(build_prediction_input(result))
        result_build_ms = _elapsed_ms(stage_start)
        trajectories = []
        prediction_infer_ms = 0.0
        prediction_export_ms = 0.0
        prediction_ms = 0.0
        if self.prediction_model is not None:
            stage_start = time.perf_counter()
            trajectories = self.prediction_model.predict(result.tracks)
            prediction_infer_ms = _elapsed_ms(stage_start)
            stage_start = time.perf_counter()
            self.predicted_trajectories.extend(
                prediction_rows(
                    frame_id=frame.frame_id,
                    timestamp_sec=frame.timestamp_sec,
                    trajectories=trajectories,
                )
            )
            prediction_export_ms = _elapsed_ms(stage_start)
            prediction_ms = prediction_infer_ms + prediction_export_ms
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
        )
        self.ttc_warnings.extend(warnings)
        warning_ms = _elapsed_ms(stage_start)
        marker_build_ms = 0.0
        marker_publish_ms = 0.0
        marker_count = 0
        if self.marker_publisher is not None:
            stage_start = time.perf_counter()
            marker_frame = self.args.marker_frame or msg.header.frame_id or "map"
            marker_array = build_tracking_marker_array(
                frame_id=marker_frame,
                timestamp=msg.header.stamp,
                tracks=result.tracks,
                trajectories=trajectories,
                warnings=warnings,
                history_tail=self.args.marker_history_tail,
                ego_delta=ego_delta,
                ego_compensation_enabled=self.args.ego_compensation,
                ego_wheelbase_m=self.args.ego_wheelbase,
                ego_steer_ratio=self.args.ego_steer_ratio,
                vehicle_front_m=self.args.vehicle_front,
                vehicle_rear_m=self.args.vehicle_rear,
                vehicle_side_m=self.args.vehicle_side,
                ego_prediction_horizon_sec=self.args.ego_marker_horizon,
                ego_prediction_step_sec=self.args.ego_marker_step,
            )
            marker_count = len(marker_array.markers)
            marker_build_ms = _elapsed_ms(stage_start)
            stage_start = time.perf_counter()
            self.marker_publisher.publish(marker_array)
            marker_publish_ms = _elapsed_ms(stage_start)
        marker_total_ms = marker_build_ms + marker_publish_ms

        stage_start = time.perf_counter()
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
        detection_export_ms = _elapsed_ms(stage_start)

        latency_row_start = time.perf_counter()
        latency_row = {
            "frame": frame_id,
            "summary": "",
            "warmup_excluded": frame_id < self.args.latency_warmup_frames,
            "msg_timestamp_sec": timestamp_sec,
            "receive_wall_sec": receive_wall_sec,
            "playback_lag_ms": self._playback_lag_ms(timestamp_sec, receive_wall_sec),
            "total_callback_ms": 0.0,
            "pointcloud_ms": pointcloud_convert_ms + pointcloud_roi_ms,
            "pointcloud_convert_ms": pointcloud_convert_ms,
            "pointcloud_roi_ms": pointcloud_roi_ms,
            "perception_ms": detector_infer_ms + detection_roi_ms,
            "detector_infer_ms": detector_infer_ms,
            "detection_roi_ms": detection_roi_ms,
            "filter_ms": filter_ms,
            "ego_motion_ms": ego_pop_ms + ego_config_ms + ego_apply_ms,
            "ego_pop_ms": ego_pop_ms,
            "ego_config_ms": ego_config_ms,
            "ego_apply_ms": ego_apply_ms,
            "tracking_ms": tracking_ms,
            "result_build_ms": result_build_ms,
            "prediction_infer_ms": prediction_infer_ms,
            "prediction_export_ms": prediction_export_ms,
            "prediction_ms": prediction_ms,
            "planner_ms": planner_ms,
            "warning_ms": warning_ms,
            "marker_ms": marker_total_ms,
            "marker_build_ms": marker_build_ms,
            "marker_publish_ms": marker_publish_ms,
            "marker_total_ms": marker_total_ms,
            "detection_export_ms": detection_export_ms,
            "latency_row_ms": 0.0,
            "unaccounted_ms": 0.0,
            "points": len(points),
            "raw_points": len(raw_points),
            "points_after_roi": len(points),
            "raw_detections": len(raw_detections),
            "detections_after_roi": len(detections),
            "detections": len(detections),
            "pedestrians": len(pedestrian_detections),
            "tracks": len(tracks),
            "predicted": len(trajectories),
            "active_warnings": len([item for item in warnings if item.level > 0]),
            "markers": marker_count,
            "ego_motion_valid": ego_delta.valid,
            "ego_motion_reset": ego_delta.reset,
            "ego_speed_mps": ego_delta.speed_mps,
            "ego_steering_deg": ego_delta.steering_deg,
            "ego_brake_pressed": ego_delta.brake_pressed,
            "ego_brake_lights": ego_delta.brake_lights,
            "ego_delta_x_m": ego_delta.dx_m,
            "ego_delta_y_m": ego_delta.dy_m,
            "ego_delta_yaw_rad": ego_delta.dyaw_rad,
        }
        self.latency_rows.append(latency_row)
        latency_row["latency_row_ms"] = _elapsed_ms(latency_row_start)
        total_ms = _elapsed_ms(callback_start)
        latency_row["total_callback_ms"] = total_ms
        accounted_ms = sum(
            float(latency_row[field])
            for field in (
                "pointcloud_convert_ms",
                "pointcloud_roi_ms",
                "detector_infer_ms",
                "detection_roi_ms",
                "filter_ms",
                "ego_pop_ms",
                "ego_config_ms",
                "ego_apply_ms",
                "tracking_ms",
                "result_build_ms",
                "prediction_infer_ms",
                "prediction_export_ms",
                "planner_ms",
                "warning_ms",
                "marker_build_ms",
                "marker_publish_ms",
                "detection_export_ms",
                "latency_row_ms",
            )
        )
        latency_row["unaccounted_ms"] = total_ms - accounted_ms

        if frame_id % self.args.print_every == 0:
            playback_lag_ms = self._playback_lag_ms(timestamp_sec, receive_wall_sec)
            ego_text = ""
            if self.args.ego_compensation:
                ego_text = (
                    f" ego_v={ego_delta.speed_mps:.2f}mps ego_dx={ego_delta.dx_m:.3f}m "
                    f"ego_yaw={ego_delta.dyaw_rad:.4f}rad ego_brake={ego_delta.brake_pressed} "
                    f"ego_reset={ego_delta.reset}"
                )
            self.get_logger().info(
                f"frame={frame_id} points={len(points)} detections={len(detections)} "
                f"pedestrians={len(pedestrian_detections)} tracks={len(tracks)} "
                f"predicted={len(trajectories)} warnings={len([item for item in warnings if item.level > 0])} "
                f"latency_total={total_ms:.1f}ms detector={detector_infer_ms:.1f}ms "
                f"prediction={prediction_ms:.1f}ms marker={marker_total_ms:.1f}ms "
                f"replay_lag={playback_lag_ms:.1f}ms"
                f"{ego_text}"
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
        write_latency_csv(
            self.latency_rows,
            output_dir / "latency.csv",
            warmup_frames=self.args.latency_warmup_frames,
        )
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
                vehicle_front_m=args.vehicle_front,
                vehicle_rear_m=args.vehicle_rear,
                vehicle_side_m=args.vehicle_side,
                low_speed_suppress_mps=args.ttc_low_speed_kph / 3.6,
                brake_ttc_scale=args.ttc_brake_threshold_scale,
            )
        )

    def _build_ego_reader(self, args: argparse.Namespace) -> PandaEgoMotionReader | None:
        if not args.ego_compensation:
            return None
        self.get_logger().info(
            f"Starting Panda ego motion reader on bus {args.ego_can_bus} "
            f"(wheelbase={args.ego_wheelbase:.3f}m steer_ratio={args.ego_steer_ratio:.2f})"
        )
        reader = PandaEgoMotionReader(
            bus=args.ego_can_bus,
            can_speed=args.ego_can_speed,
            data_speed=args.ego_data_speed,
            configure_panda=not args.ego_no_config,
            wheelbase_m=args.ego_wheelbase,
            steer_ratio=args.ego_steer_ratio,
            angle_source=args.ego_angle_source,
            invert_steer=args.ego_invert_steer,
            max_dt_sec=args.ego_max_dt,
            stop_speed_threshold_mps=args.ego_stop_speed_threshold,
            stop_reset_sec=args.ego_stop_reset_sec,
        )
        if reader.wait_ready(timeout_sec=2.0):
            if reader.error:
                self.get_logger().warning(f"Panda ego motion reader failed: {reader.error}")
                reader.stop()
                return None
            self.get_logger().info("Panda ego motion reader ready.")
        else:
            self.get_logger().warning("Panda ego motion reader is still connecting; continuing without initial samples.")
        return reader

    def _pop_ego_motion_delta(self) -> EgoMotionDelta:
        if self.ego_reader is None:
            if self.args.ego_compensation:
                return EgoMotionDelta(speed_mps=0.0)
            return EgoMotionDelta(speed_mps=self.args.ego_speed)
        return self.ego_reader.pop_delta()

    def _playback_lag_ms(self, timestamp_sec: float, receive_wall_sec: float) -> float:
        if self.first_msg_timestamp_sec is None or self.first_receive_wall_sec is None:
            return 0.0
        msg_elapsed = timestamp_sec - self.first_msg_timestamp_sec
        wall_elapsed = receive_wall_sec - self.first_receive_wall_sec
        expected_wall_elapsed = msg_elapsed / max(self.args.latency_playback_rate, 1e-6)
        return (wall_elapsed - expected_wall_elapsed) * 1000.0

    def close(self) -> None:
        if self.ego_reader is not None:
            self.ego_reader.stop()
            self.ego_reader = None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to ROS2 PointCloud2, run OpenPCDet perception, and track pedestrians"
    )
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--perception", choices=["dsvt", "pointpillar"], default="pointpillar")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--perception-roi-m", type=float, default=10.0, help="Radial perception ROI in meters; <=0 disables cropping")
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
    parser.add_argument(
        "--latency-warmup-frames",
        type=int,
        default=5,
        help="Exclude the first N frames from latency average and percent summary rows",
    )
    parser.add_argument("--ego-speed", type=float, default=10.0)
    parser.add_argument("--ego-compensation", action="store_true", help="Use Panda wheel speed/steering for ego motion compensation")
    parser.add_argument("--ego-can-bus", type=int, default=0, help="Panda bus with WHEEL_SPEEDS/STEERING_SENSORS")
    parser.add_argument("--ego-can-speed", type=int, default=500, help="Ego CAN nominal speed in kbps")
    parser.add_argument("--ego-data-speed", type=int, default=2000, help="Ego CAN-FD data speed in kbps")
    parser.add_argument("--ego-no-config", action="store_true", help="Do not configure Panda CAN-FD speeds for ego reader")
    parser.add_argument("--ego-wheelbase", type=float, default=2.900)
    parser.add_argument("--ego-steer-ratio", type=float, default=16.0)
    parser.add_argument("--ego-angle-source", choices=("sensor", "mdps"), default="sensor")
    parser.add_argument("--ego-invert-steer", action="store_true")
    parser.add_argument("--ego-max-dt", type=float, default=0.1)
    parser.add_argument("--ego-stop-speed-threshold", type=float, default=0.05)
    parser.add_argument("--ego-stop-reset-sec", type=float, default=0.7)
    parser.add_argument(
        "--ego-reset-tracks-on-stop",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reset tracker when the vehicle remains stopped long enough to cut accumulated drift",
    )
    parser.add_argument("--safety-radius", type=float, default=1.0)
    parser.add_argument(
        "--vehicle-front",
        type=float,
        default=2.40,
        help="Forward vehicle footprint from LiDAR origin in meters",
    )
    parser.add_argument(
        "--vehicle-rear",
        type=float,
        default=2.10,
        help="Rearward vehicle footprint from LiDAR origin in meters",
    )
    parser.add_argument("--vehicle-side", type=float, default=1.00, help="Lateral vehicle footprint from LiDAR origin in meters")
    parser.add_argument(
        "--ttc-low-speed-kph",
        type=float,
        default=10.0,
        help="Suppress TTC warnings at or below this ego speed in km/h",
    )
    parser.add_argument(
        "--ttc-brake-threshold-scale",
        type=float,
        default=0.70,
        help="Scale TTC warning thresholds while driver brake is pressed; lower is tighter",
    )
    parser.add_argument("--marker-topic", default="/adas/tracking_markers")
    parser.add_argument("--marker-frame", default=None)
    parser.add_argument("--marker-history-tail", type=int, default=20)
    parser.add_argument("--ego-marker-horizon", type=float, default=3.0, help="Seconds of predicted ego path to draw in RViz")
    parser.add_argument("--ego-marker-step", type=float, default=0.2, help="Time step for RViz ego path markers")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def _elapsed_ms(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def _filter_points_by_roi(points, roi_m: float):
    if roi_m <= 0.0 or len(points) == 0:
        return points
    roi_sq = roi_m * roi_m
    mask = (points[:, 0] * points[:, 0]) + (points[:, 1] * points[:, 1]) <= roi_sq
    return points[mask]


def _filter_detections_by_roi(detections, roi_m: float):
    if roi_m <= 0.0:
        return detections
    roi_sq = roi_m * roi_m
    return [
        det
        for det in detections
        if (det.x * det.x) + (det.y * det.y) <= roi_sq
    ]


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
        node.close()
        node.destroy_node()


if __name__ == "__main__":
    main()
