from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import asdict, replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent.parent))

from app.core.config import DEFAULT_STATIC_ROI_Z_MAX_M, DEFAULT_STATIC_ROI_Z_MIN_M, PipelineConfig
from app.ego_motion import (
    DEFAULT_STEER_RATIO,
    EgoMotionDelta,
    PandaEgoMotionReader,
)
from app.control.ttc_warning import (
    StaticObstacleObservation,
    TTCWarning,
    TTCWarningAdapter,
    TTCWarningConfig,
    write_warnings_json,
)
from app.core.domain_types import (
    FrameInput,
    PredictedTrajectory,
    TrackingFrameResult,
    TrackedPedestrian,
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
from app.prediction.base import PredictionModel
from app.prediction.input_builder import build_prediction_input
from app.visualization.dashboard_client import DashboardPublisher
from app.visualization.rviz_markers import build_tracking_marker_array

PROJECT_ROOT = Path(__file__).resolve().parent.parent


class PredictionHoldModel(PredictionModel):
    """Keep recent predictions visible for active tracks through short output drops."""

    def __init__(
        self,
        base_model: PredictionModel,
        hold_frames: int,
        confidence_decay: float,
    ):
        self.base_model = base_model
        self.hold_frames = max(0, hold_frames)
        self.confidence_decay = min(max(confidence_decay, 0.0), 1.0)
        self.frame_index = 0
        self._cache: dict[int, _HeldPrediction] = {}

    def predict(
        self,
        tracked_objects: list[TrackedPedestrian],
        timestamp_sec: float | None = None,
        ego_motion_delta: tuple[float, float, float] | None = None,
    ) -> list[PredictedTrajectory]:
        self.frame_index += 1
        fresh = self.base_model.predict(
            tracked_objects,
            timestamp_sec=timestamp_sec,
            ego_motion_delta=ego_motion_delta,
        )
        fresh_by_id = {trajectory.track_id: trajectory for trajectory in fresh}
        tracks_by_id = {track.track_id: track for track in tracked_objects}

        for trajectory in fresh:
            track = tracks_by_id.get(trajectory.track_id)
            if track is None:
                continue
            self._cache[trajectory.track_id] = _HeldPrediction(
                trajectory=trajectory,
                anchor_x=float(track.x),
                anchor_y=float(track.y),
                frame_index=self.frame_index,
            )

        active_ids = set(tracks_by_id)
        for track_id in list(self._cache):
            if track_id not in active_ids:
                del self._cache[track_id]

        output = list(fresh)
        for track_id, track in tracks_by_id.items():
            if track_id in fresh_by_id:
                continue
            held = self._cache.get(track_id)
            if held is None:
                continue

            age_frames = self.frame_index - held.frame_index
            if age_frames > self.hold_frames:
                continue

            output.append(self._translate_held_prediction(track, held, age_frames))

        return output

    def _translate_held_prediction(
        self,
        track: TrackedPedestrian,
        held: "_HeldPrediction",
        age_frames: int,
    ) -> PredictedTrajectory:
        dx = float(track.x) - held.anchor_x
        dy = float(track.y) - held.anchor_y
        confidence = held.trajectory.confidence * (self.confidence_decay ** age_frames)
        return replace(
            held.trajectory,
            points=[
                replace(point, x=point.x + dx, y=point.y + dy)
                for point in held.trajectory.points
            ],
            confidence=confidence,
            model_name=f"{held.trajectory.model_name}+hold",
        )


class _HeldPrediction:
    def __init__(
        self,
        trajectory: PredictedTrajectory,
        anchor_x: float,
        anchor_y: float,
        frame_index: int,
    ):
        self.trajectory = trajectory
        self.anchor_x = anchor_x
        self.anchor_y = anchor_y
        self.frame_index = frame_index


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
        self.dashboard_publisher = None
        self.dashboard_error_reported = False

        config = PipelineConfig(
            perception_name=f"openpcdet_{args.perception}",
            perception_score_threshold=args.score_threshold,
            perception_device=args.device,
            pedestrian_min_point_max_distance_m=args.pedestrian_min_point_max_distance,
            pedestrian_max_point_max_distance_m=args.pedestrian_max_point_max_distance,
        )
        self.get_logger().info(f"Loading OpenPCDet {args.perception} model...")
        self.pipeline = RealTimePedestrianTrackingPipeline(config)
        self.get_logger().info("Model loaded. Waiting for PointCloud2 frames.")
        self.prediction_model = self._build_prediction_model(args, config)
        self.warning_adapter = self._build_warning_adapter(args)
        self._topic_ego_delta = EgoMotionDelta()
        self._topic_ego_pending_dx_m = 0.0
        self._topic_ego_pending_dy_m = 0.0
        self._topic_ego_pending_dyaw_rad = 0.0
        self._topic_ego_reset_pending = False
        self._topic_ego_has_sample = False
        self.ego_topic_publisher = None
        if args.ego_topic and args.ego_source != "topic":
            self.ego_topic_publisher = self.create_publisher(String, args.ego_topic, 100)
            self.get_logger().info(f"Publishing ego motion JSON on {args.ego_topic}")
        self.ego_topic_subscription = None
        if args.ego_compensation and args.ego_source == "topic":
            self.ego_topic_subscription = self.create_subscription(
                String,
                args.ego_input_topic,
                self._on_ego_motion_topic,
                100,
            )
            self.get_logger().info(f"Using ego motion from ROS topic {args.ego_input_topic}")
        self.ego_raw_can_publisher = None
        if args.ego_raw_topic and args.ego_source == "panda":
            self.ego_raw_can_publisher = self.create_publisher(String, args.ego_raw_topic, 1000)
            self.get_logger().info(f"Publishing raw ego CAN JSON on {args.ego_raw_topic}")
        self.ego_reader = self._build_ego_reader(args)
        self.marker_publisher = None
        if not args.no_rviz:
            self.marker_publisher = self.create_publisher(MarkerArray, args.marker_topic, 10)
            self.get_logger().info(f"Publishing RViz markers on {args.marker_topic}")
        if args.dashboard_url:
            self.dashboard_publisher = DashboardPublisher(
                url=args.dashboard_url,
                timeout_sec=args.dashboard_timeout_sec,
                max_queue=args.dashboard_queue_size,
            )
            self.get_logger().info(f"Publishing browser dashboard frames to {args.dashboard_url}")

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
        points = raw_points
        pointcloud_roi_ms = 0.0
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            sensor_source=self.args.topic,
            payload={"points": points},
        )

        stage_start = time.perf_counter()
        raw_detections = self.pipeline.detector.infer(frame)
        detector_infer_ms = _elapsed_ms(stage_start)
        detections = raw_detections
        detection_roi_ms = 0.0
        stage_start = time.perf_counter()
        pedestrian_detections = filter_pedestrians(
            detections,
            point_spread_filter=self.pipeline.pedestrian_point_spread_filter,
        )
        filter_ms = _elapsed_ms(stage_start)

        stage_start = time.perf_counter()
        ego_delta = self._pop_ego_motion_delta()
        ego_pop_ms = _elapsed_ms(stage_start)
        self._publish_ego_motion_topic(frame_id, timestamp_sec, ego_delta)
        stage_start = time.perf_counter()
        warning_config_updates = _load_runtime_static_roi_config(self.args.roi_control_file, self.args)
        if ego_delta.valid:
            warning_config_updates.update(
                {
                    "ego_speed_mps": ego_delta.speed_mps,
                    "ego_steering_deg": ego_delta.steering_deg,
                    "driver_brake_pressed": ego_delta.brake_pressed,
                    "driver_accelerator_pressed": ego_delta.accelerator_pressed,
                }
            )
        self.warning_adapter.config = replace(self.warning_adapter.config, **warning_config_updates)
        ego_config_ms = _elapsed_ms(stage_start)
        stage_start = time.perf_counter()
        if ego_delta.valid or ego_delta.reset:
            self.pipeline.tracker.apply_ego_motion(
                dx_m=ego_delta.dx_m,
                dy_m=ego_delta.dy_m,
                dyaw_rad=ego_delta.dyaw_rad,
                reset=ego_delta.reset,
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
            prediction_ego_delta = (
                (ego_delta.dx_m, ego_delta.dy_m, ego_delta.dyaw_rad)
                if ego_delta.valid
                else None
            )
            trajectories = self.prediction_model.predict(
                result.tracks,
                timestamp_sec=frame.timestamp_sec,
                ego_motion_delta=prediction_ego_delta,
            )
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
            points=raw_points,
        )
        static_obstacle = self.warning_adapter.latest_static_obstacle
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
                static_obstacle=static_obstacle,
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
            "ego_accelerator_pressed": ego_delta.accelerator_pressed,
            "ego_accelerator_pedal": ego_delta.accelerator_pedal,
            "ego_accelerator_pedal_raw": ego_delta.accelerator_pedal_raw,
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
        self._publish_dashboard_frame(result, trajectories, warnings, static_obstacle, latency_row, ego_delta, raw_points)

        if frame_id % self.args.print_every == 0:
            playback_lag_ms = self._playback_lag_ms(timestamp_sec, receive_wall_sec)
            ego_text = ""
            if self.args.ego_compensation:
                ego_text = (
                    f" ego_v={ego_delta.speed_mps:.2f}mps ego_dx={ego_delta.dx_m:.3f}m "
                    f"ego_yaw={ego_delta.dyaw_rad:.4f}rad ego_brake={ego_delta.brake_pressed} "
                    f"ego_accel={ego_delta.accelerator_pressed} "
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
                smooth_alpha=args.prediction_smooth_alpha,
                max_pedestrian_speed=args.prediction_max_speed,
            )
            self.get_logger().info("SR-LSTM prediction model loaded.")
            if args.prediction_hold:
                self.get_logger().info(
                    "Prediction hold enabled: "
                    f"frames={args.prediction_hold_frames}, "
                    f"confidence_decay={args.prediction_hold_confidence_decay:.2f}"
                )
                return PredictionHoldModel(
                    base_model=model,
                    hold_frames=args.prediction_hold_frames,
                    confidence_decay=args.prediction_hold_confidence_decay,
                )
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
                perception_low_speed_suppress_mps=args.ttc_perception_low_speed_kph / 3.6,
                brake_ttc_scale=args.ttc_brake_threshold_scale,
                roi_x_min=args.roi_x_min,
                roi_x_max=args.roi_x_max,
                roi_y_min=args.roi_y_min,
                roi_y_max=args.roi_y_max,
                roi_z_min=args.roi_z_min,
                roi_z_max=args.roi_z_max,
                static_obstacle_min_points=args.static_min_points,
            )
        )

    def _build_ego_reader(self, args: argparse.Namespace) -> PandaEgoMotionReader | None:
        if not args.ego_compensation or args.ego_source != "panda":
            return None
        self.get_logger().info(
            f"Starting Panda ego motion reader on bus {args.ego_can_bus} "
            f"(wheelbase={args.ego_wheelbase:.3f}m steer_ratio={args.ego_steer_ratio:.2f} "
            f"steer_bias={args.ego_steer_bias_deg:.2f}deg)"
        )
        reader = PandaEgoMotionReader(
            bus=args.ego_can_bus,
            can_speed=args.ego_can_speed,
            data_speed=args.ego_data_speed,
            configure_panda=not args.ego_no_config,
            wheelbase_m=args.ego_wheelbase,
            steer_ratio=args.ego_steer_ratio,
            steer_bias_deg=args.ego_steer_bias_deg,
            angle_source=args.ego_angle_source,
            invert_steer=args.ego_invert_steer,
            max_dt_sec=args.ego_max_dt,
            raw_can_callback=self._publish_raw_ego_can if args.ego_raw_topic else None,
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
        if self.args.ego_compensation and self.args.ego_source == "topic":
            delta = self._topic_ego_delta
            if self._topic_ego_has_sample:
                delta = EgoMotionDelta(
                    dx_m=self._topic_ego_pending_dx_m,
                    dy_m=self._topic_ego_pending_dy_m,
                    dyaw_rad=self._topic_ego_pending_dyaw_rad,
                    speed_mps=delta.speed_mps,
                    steering_deg=delta.steering_deg,
                    accelerator_pressed=delta.accelerator_pressed,
                    accelerator_pedal=delta.accelerator_pedal,
                    accelerator_pedal_raw=delta.accelerator_pedal_raw,
                    brake_pressed=delta.brake_pressed,
                    brake_lights=delta.brake_lights,
                    valid=delta.valid,
                    reset=self._topic_ego_reset_pending,
                )
            self._topic_ego_pending_dx_m = 0.0
            self._topic_ego_pending_dy_m = 0.0
            self._topic_ego_pending_dyaw_rad = 0.0
            self._topic_ego_reset_pending = False
            return delta
        if self.ego_reader is None:
            if self.args.ego_compensation:
                return EgoMotionDelta(speed_mps=0.0)
            return EgoMotionDelta(speed_mps=self.args.ego_speed)
        return self.ego_reader.pop_delta()

    def _on_ego_motion_topic(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        reset = bool(data.get("reset", False))
        if reset:
            self._topic_ego_pending_dx_m = 0.0
            self._topic_ego_pending_dy_m = 0.0
            self._topic_ego_pending_dyaw_rad = 0.0
            self._topic_ego_reset_pending = True
        else:
            self._topic_ego_pending_dx_m += float(data.get("dx_m", 0.0) or 0.0)
            self._topic_ego_pending_dy_m += float(data.get("dy_m", 0.0) or 0.0)
            self._topic_ego_pending_dyaw_rad += float(data.get("dyaw_rad", 0.0) or 0.0)

        self._topic_ego_delta = EgoMotionDelta(
            dx_m=0.0,
            dy_m=0.0,
            dyaw_rad=0.0,
            speed_mps=float(data.get("speed_mps", 0.0) or 0.0),
            steering_deg=float(data.get("steering_deg", 0.0) or 0.0),
            accelerator_pressed=bool(data.get("accelerator_pressed", False)),
            accelerator_pedal=float(data.get("accelerator_pedal", 0.0) or 0.0),
            accelerator_pedal_raw=int(data.get("accelerator_pedal_raw", 0) or 0),
            brake_pressed=bool(data.get("brake_pressed", False)),
            brake_lights=bool(data.get("brake_lights", data.get("brake_pressed", False))),
            valid=bool(data.get("valid", True)),
            reset=reset,
        )
        self._topic_ego_has_sample = True

    def _publish_ego_motion_topic(
        self,
        frame_id: int,
        timestamp_sec: float,
        ego_delta: EgoMotionDelta,
    ) -> None:
        if self.ego_topic_publisher is None:
            return
        payload = {
            "frame_id": frame_id,
            "timestamp_sec": timestamp_sec,
            **asdict(ego_delta),
        }
        self.ego_topic_publisher.publish(String(data=json.dumps(_json_safe(payload), separators=(",", ":"))))

    def _publish_raw_ego_can(self, payload: dict) -> None:
        if self.ego_raw_can_publisher is None:
            return
        self.ego_raw_can_publisher.publish(
            String(data=json.dumps(_json_safe(payload), separators=(",", ":")))
        )

    def _playback_lag_ms(self, timestamp_sec: float, receive_wall_sec: float) -> float:
        if self.first_msg_timestamp_sec is None or self.first_receive_wall_sec is None:
            return 0.0
        msg_elapsed = timestamp_sec - self.first_msg_timestamp_sec
        wall_elapsed = receive_wall_sec - self.first_receive_wall_sec
        expected_wall_elapsed = msg_elapsed / max(self.args.latency_playback_rate, 1e-6)
        return (wall_elapsed - expected_wall_elapsed) * 1000.0

    def _publish_dashboard_frame(
        self,
        result: TrackingFrameResult,
        trajectories,
        warnings: list[TTCWarning],
        static_obstacle: StaticObstacleObservation | None,
        latency_row: dict,
        ego_delta: EgoMotionDelta,
        pointcloud_points,
    ) -> None:
        if self.dashboard_publisher is None:
            return

        payload = {
            "frame_id": result.frame_id,
            "timestamp_sec": result.timestamp_sec,
            "ego_speed_mps": ego_delta.speed_mps,
            "ego_steering_deg": ego_delta.steering_deg,
            "ego_accelerator_pressed": ego_delta.accelerator_pressed,
            "ego_accelerator_pedal": ego_delta.accelerator_pedal,
            "ego_accelerator_pedal_raw": ego_delta.accelerator_pedal_raw,
            "ego_brake_pressed": ego_delta.brake_pressed,
            "ego_brake_lights": ego_delta.brake_lights,
            "safety_radius_m": self.args.safety_radius,
            "tracks": [asdict(track) for track in result.tracks],
            "trajectories": [asdict(trajectory) for trajectory in trajectories],
            "warnings": [asdict(warning) for warning in warnings],
            "static_obstacle": asdict(static_obstacle) if static_obstacle is not None else None,
            "roi_config": _load_dashboard_roi_config(self.args.roi_control_file, self.args),
            "pointcloud": _sample_pointcloud_for_dashboard(
                pointcloud_points,
                max_points=self.args.dashboard_pointcloud_max_points,
            ),
            "latency": latency_row,
        }
        self.dashboard_publisher.publish(_json_safe(payload))

        if self.dashboard_publisher.last_error and not self.dashboard_error_reported:
            self.get_logger().warning(
                f"Dashboard publish failed once: {self.dashboard_publisher.last_error}"
            )
            self.dashboard_error_reported = True

    def close(self) -> None:
        if self.dashboard_publisher is not None:
            self.dashboard_publisher.close()
            self.dashboard_publisher = None
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
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--queue-size", type=int, default=1)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--prediction", choices=["none", "srlstm"], default="none")
    parser.add_argument("--prediction-fps", type=float, default=2.5)
    parser.add_argument(
        "--prediction-hold",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Keep recent SR-LSTM predictions visible for active tracks through short output drops",
    )
    parser.add_argument(
        "--prediction-hold-frames",
        type=int,
        default=8,
        help="Maximum callback frames to keep a missing prediction visible",
    )
    parser.add_argument(
        "--prediction-hold-confidence-decay",
        type=float,
        default=0.85,
        help="Per-frame confidence multiplier for held predictions",
    )
    parser.add_argument(
        "--prediction-smooth-alpha",
        type=float,
        default=0.4,
        help="EMA smoothing factor for predicted trajectories (0=full smooth, 1=no smooth)",
    )
    parser.add_argument(
        "--prediction-max-speed",
        type=float,
        default=3.0,
        help="Max pedestrian speed for trajectory clamping in m/s",
    )
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
    parser.add_argument("--ego-speed", type=float, default=6.0)
    parser.add_argument("--ego-compensation", action="store_true", help="Use Panda wheel speed/steering for ego motion compensation")
    parser.add_argument(
        "--ego-source",
        choices=("panda", "topic"),
        default="panda",
        help="Use live Panda CAN or an existing ROS ego motion topic for ego compensation",
    )
    parser.add_argument("--ego-input-topic", default="/vehicle/ego_motion", help="Input ego motion JSON topic when --ego-source=topic")
    parser.add_argument("--ego-topic", default="/vehicle/ego_motion", help="Publish ego motion JSON to this ROS topic; empty disables it")
    parser.add_argument("--ego-raw-topic", default="/vehicle/can/raw", help="Publish raw ego CAN JSON to this ROS topic; empty disables it")
    parser.add_argument("--ego-can-bus", type=int, default=0, help="Panda bus with WHEEL_SPEEDS/STEERING_SENSORS")
    parser.add_argument("--ego-can-speed", type=int, default=500, help="Ego CAN nominal speed in kbps")
    parser.add_argument("--ego-data-speed", type=int, default=2000, help="Ego CAN-FD data speed in kbps")
    parser.add_argument("--ego-no-config", action="store_true", help="Do not configure Panda CAN-FD speeds for ego reader")
    parser.add_argument("--ego-wheelbase", type=float, default=2.900)
    parser.add_argument("--ego-steer-ratio", type=float, default=DEFAULT_STEER_RATIO, help="Steering wheel angle / road wheel angle")
    parser.add_argument(
        "--ego-steer-bias-deg",
        type=float,
        default=0.0,
        help="Steering wheel angle bias in degrees to subtract after sign correction",
    )
    parser.add_argument("--ego-angle-source", choices=("sensor", "mdps"), default="sensor")
    parser.add_argument("--ego-invert-steer", action="store_true")
    parser.add_argument("--ego-max-dt", type=float, default=0.1)
    parser.add_argument("--safety-radius", type=float, default=0.7)
    parser.add_argument("--roi-x-min", type=float, default=2.5)
    parser.add_argument("--roi-x-max", type=float, default=15.0)
    parser.add_argument("--roi-y-min", type=float, default=-1.1)
    parser.add_argument("--roi-y-max", type=float, default=1.1)
    parser.add_argument("--roi-z-min", type=float, default=DEFAULT_STATIC_ROI_Z_MIN_M)
    parser.add_argument("--roi-z-max", type=float, default=DEFAULT_STATIC_ROI_Z_MAX_M)
    parser.add_argument("--static-min-points", type=int, default=15)
    parser.add_argument(
        "--vehicle-front",
        type=float,
        default=3.50,
        help="Forward vehicle footprint from LiDAR origin in meters",
    )
    parser.add_argument(
        "--vehicle-rear",
        type=float,
        default=2.10,
        help="Rearward vehicle footprint from LiDAR origin in meters",
    )
    parser.add_argument("--vehicle-side", type=float, default=1.0, help="Lateral vehicle footprint from LiDAR origin in meters")
    parser.add_argument(
        "--ttc-low-speed-kph",
        type=float,
        default=10.0,
        help="Compatibility option; TTC level scaling is currently disabled",
    )
    parser.add_argument(
        "--ttc-perception-low-speed-kph",
        type=float,
        default=5.0,
        help="Compatibility option; TTC level scaling is currently disabled",
    )
    parser.add_argument(
        "--ttc-brake-threshold-scale",
        type=float,
        default=0.70,
        help="Compatibility option; TTC level scaling is currently disabled",
    )
    parser.add_argument("--marker-topic", default="/adas/tracking_markers")
    parser.add_argument("--marker-frame", default=None)
    parser.add_argument("--marker-history-tail", type=int, default=20)
    parser.add_argument("--ego-marker-horizon", type=float, default=3.0, help="Seconds of predicted ego path to draw in RViz")
    parser.add_argument("--ego-marker-step", type=float, default=0.2, help="Time step for RViz ego path markers")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument(
        "--dashboard-url",
        default=None,
        help="Optional browser dashboard endpoint, e.g. http://localhost:8000/api/frame",
    )
    parser.add_argument("--dashboard-timeout-sec", type=float, default=0.03)
    parser.add_argument("--dashboard-queue-size", type=int, default=2)
    parser.add_argument("--dashboard-pointcloud-max-points", type=int, default=1800)
    parser.add_argument(
        "--roi-control-file",
        type=Path,
        default=PROJECT_ROOT / "artifacts" / "runtime_roi_config.json",
        help="Dashboard ROI JSON file used for runtime static-obstacle ROI",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser


def _elapsed_ms(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def _load_dashboard_roi_config(path: Path | None, args: argparse.Namespace) -> dict:
    static_roi = _load_runtime_static_roi_config(path, args)
    config = {}
    config.update(
        {
            "static_roi_x_min": static_roi["roi_x_min"],
            "static_roi_x_max": static_roi["roi_x_max"],
            "static_roi_y_min": static_roi["roi_y_min"],
            "static_roi_y_max": static_roi["roi_y_max"],
            "static_roi_z_min": static_roi["roi_z_min"],
            "static_roi_z_max": static_roi["roi_z_max"],
            "vehicle_front_m": float(args.vehicle_front),
            "vehicle_rear_m": float(args.vehicle_rear),
            "vehicle_side_m": float(args.vehicle_side),
        }
    )
    return config


def _load_runtime_static_roi_config(path: Path | None, args: argparse.Namespace) -> dict:
    config = {
        "roi_x_min": float(args.roi_x_min),
        "roi_x_max": float(args.roi_x_max),
        "roi_y_min": float(args.roi_y_min),
        "roi_y_max": float(args.roi_y_max),
        "roi_z_min": float(args.roi_z_min),
        "roi_z_max": float(args.roi_z_max),
    }
    data = _read_runtime_roi_json(path)
    mapping = {
        "static_roi_x_min": "roi_x_min",
        "static_roi_x_max": "roi_x_max",
        "static_roi_y_min": "roi_y_min",
        "static_roi_y_max": "roi_y_max",
        "static_roi_z_min": "roi_z_min",
        "static_roi_z_max": "roi_z_max",
    }
    for json_key, config_key in mapping.items():
        value = _finite_float(data.get(json_key))
        if value is not None:
            config[config_key] = value

    if config["roi_x_min"] > config["roi_x_max"]:
        config["roi_x_min"], config["roi_x_max"] = config["roi_x_max"], config["roi_x_min"]
    if config["roi_y_min"] > config["roi_y_max"]:
        config["roi_y_min"], config["roi_y_max"] = config["roi_y_max"], config["roi_y_min"]
    if config["roi_z_min"] > config["roi_z_max"]:
        config["roi_z_min"], config["roi_z_max"] = config["roi_z_max"], config["roi_z_min"]
    return config


def _read_runtime_roi_json(path: Path | None) -> dict:
    if path is None:
        return {}
    resolved = path if path.is_absolute() else PROJECT_ROOT / path
    try:
        data = json.loads(resolved.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _finite_float(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _sample_pointcloud_for_dashboard(points, max_points: int) -> dict:
    point_count = int(len(points))
    max_points = max(int(max_points), 0)
    if point_count == 0 or max_points == 0:
        return {"points": [], "sampled": 0, "total": point_count}

    sample = points
    if point_count > max_points:
        step = math.ceil(point_count / max_points)
        sample = points[::step][:max_points]

    rows = []
    for point in sample:
        rows.append([
            round(float(point[0]), 2),
            round(float(point[1]), 2),
            round(float(point[2]), 2),
        ])
    return {"points": rows, "sampled": len(rows), "total": point_count}


def _json_safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


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
