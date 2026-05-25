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
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_latency_csv(rows: list[dict], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "frame",
        "msg_timestamp_sec",
        "receive_wall_sec",
        "playback_lag_ms",
        "total_callback_ms",
        "pointcloud_ms",
        "perception_ms",
        "filter_ms",
        "tracking_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
        "points",
        "detections",
        "pedestrians",
        "tracks",
        "predicted",
        "active_warnings",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
        writer.writerows(_latency_summary_rows(rows, fieldnames))


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


def _latency_summary_rows(rows: list[dict], fieldnames: list[str]) -> list[dict]:
    measured_rows = [row for row in rows if isinstance(row.get("frame"), int) and row["frame"] != 0]
    if not measured_rows:
        return []

    avg_row = {field: "" for field in fieldnames}
    avg_row["frame"] = "avg_excluding_frame0"
    numeric_fields = [
        "playback_lag_ms",
        "total_callback_ms",
        "pointcloud_ms",
        "perception_ms",
        "filter_ms",
        "tracking_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
        "points",
        "detections",
        "pedestrians",
        "tracks",
        "predicted",
        "active_warnings",
    ]
    for field in numeric_fields:
        avg_row[field] = round(
            sum(float(row[field]) for row in measured_rows) / len(measured_rows),
            3,
        )

    percent_row = {field: "" for field in fieldnames}
    percent_row["frame"] = "stage_percent_excluding_frame0"
    avg_total = float(avg_row["total_callback_ms"])
    percent_row["total_callback_ms"] = 100.0
    for field in [
        "pointcloud_ms",
        "perception_ms",
        "filter_ms",
        "tracking_ms",
        "prediction_ms",
        "planner_ms",
        "warning_ms",
        "marker_ms",
    ]:
        percent_row[field] = round((float(avg_row[field]) / avg_total) * 100.0, 2) if avg_total > 0.0 else 0.0

    return [avg_row, percent_row]

