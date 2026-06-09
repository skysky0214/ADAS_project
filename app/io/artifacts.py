from __future__ import annotations

import csv
import json
from pathlib import Path

from app.core.domain_types import PredictedTrajectory


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
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_latency_csv(rows: list[dict], csv_path: Path, warmup_frames: int = 5) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame",
        "summary",
        "warmup_excluded",
        "msg_timestamp_sec",
        "receive_wall_sec",
        "playback_lag_ms",
        "total_callback_ms",
        "pointcloud_ms",
        "pointcloud_convert_ms",
        "pointcloud_roi_ms",
        "perception_ms",
        "detector_infer_ms",
        "detection_roi_ms",
        "filter_ms",
        "ego_motion_ms",
        "ego_pop_ms",
        "ego_config_ms",
        "ego_apply_ms",
        "tracking_ms",
        "result_build_ms",
        "prediction_infer_ms",
        "prediction_export_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
        "marker_build_ms",
        "marker_publish_ms",
        "marker_total_ms",
        "detection_export_ms",
        "latency_row_ms",
        "unaccounted_ms",
        "points",
        "raw_points",
        "points_after_roi",
        "raw_detections",
        "detections_after_roi",
        "detections",
        "pedestrians",
        "tracks",
        "predicted",
        "active_warnings",
        "markers",
        "ego_motion_valid",
        "ego_motion_reset",
        "ego_speed_mps",
        "ego_steering_deg",
        "ego_delta_x_m",
        "ego_delta_y_m",
        "ego_delta_yaw_rad",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        writer.writerows(_latency_summary_rows(rows, fieldnames, warmup_frames))


def prediction_rows(
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
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")


def _latency_summary_rows(rows: list[dict], fieldnames: list[str], warmup_frames: int) -> list[dict]:
    measured_rows = [
        row
        for row in rows
        if isinstance(row.get("frame"), int) and row["frame"] >= warmup_frames
    ]
    if not measured_rows:
        return []

    avg_row = {field: "" for field in fieldnames}
    avg_row["frame"] = "avg"
    avg_row["summary"] = f"avg_excluding_first_{warmup_frames}_frames"
    avg_row["warmup_excluded"] = warmup_frames
    numeric_fields = [
        "playback_lag_ms",
        "total_callback_ms",
        "pointcloud_ms",
        "pointcloud_convert_ms",
        "pointcloud_roi_ms",
        "perception_ms",
        "detector_infer_ms",
        "detection_roi_ms",
        "filter_ms",
        "ego_motion_ms",
        "ego_pop_ms",
        "ego_config_ms",
        "ego_apply_ms",
        "tracking_ms",
        "result_build_ms",
        "prediction_infer_ms",
        "prediction_export_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
        "marker_build_ms",
        "marker_publish_ms",
        "marker_total_ms",
        "detection_export_ms",
        "latency_row_ms",
        "unaccounted_ms",
        "points",
        "raw_points",
        "points_after_roi",
        "raw_detections",
        "detections_after_roi",
        "detections",
        "pedestrians",
        "tracks",
        "predicted",
        "active_warnings",
        "markers",
        "ego_speed_mps",
        "ego_steering_deg",
        "ego_delta_x_m",
        "ego_delta_y_m",
        "ego_delta_yaw_rad",
    ]
    for field in numeric_fields:
        avg_row[field] = round(
            sum(float(row.get(field, 0.0)) for row in measured_rows) / len(measured_rows),
            3,
        )

    percent_row = {field: "" for field in fieldnames}
    percent_row["frame"] = "percent"
    percent_row["summary"] = f"stage_percent_of_avg_total_excluding_first_{warmup_frames}_frames"
    percent_row["warmup_excluded"] = warmup_frames
    avg_total = float(avg_row["total_callback_ms"])
    percent_row["total_callback_ms"] = 100.0
    for field in [
        "pointcloud_ms",
        "pointcloud_convert_ms",
        "pointcloud_roi_ms",
        "perception_ms",
        "detector_infer_ms",
        "detection_roi_ms",
        "filter_ms",
        "ego_motion_ms",
        "ego_pop_ms",
        "ego_config_ms",
        "ego_apply_ms",
        "tracking_ms",
        "result_build_ms",
        "prediction_infer_ms",
        "prediction_export_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
        "marker_build_ms",
        "marker_publish_ms",
        "marker_total_ms",
        "detection_export_ms",
        "latency_row_ms",
        "unaccounted_ms",
    ]:
        percent_row[field] = round((float(avg_row[field]) / avg_total) * 100.0, 2) if avg_total > 0.0 else 0.0

    return [avg_row, percent_row]
