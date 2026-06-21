from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parents[2]))

from app.core.config import PipelineConfig
from app.core.domain_types import FrameInput, TrackingFrameResult
from app.core.logging_utils import (
    write_prediction_input_json,
    write_tracking_csv,
    write_tracking_json,
)
from app.perception.pedestrian_filter import filter_pedestrians
from app.pipeline import RealTimePedestrianTrackingPipeline
from app.prediction.input_builder import build_prediction_input


def _pointcloud2_to_xyzi(msg: PointCloud2) -> np.ndarray:
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


def _iter_lidar_frames(
    bag_path: Path,
    topic_name: str,
    start_frame: int,
    max_frames: int,
):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_frame_idx = 0
    first_timestamp_ns: int | None = None
    yielded = 0
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != topic_name:
            continue
        if topic_frame_idx < start_frame:
            topic_frame_idx += 1
            continue
        if yielded >= max_frames:
            break

        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        msg = deserialize_message(data, PointCloud2)
        points = _pointcloud2_to_xyzi(msg)
        timestamp_sec = (timestamp_ns - first_timestamp_ns) * 1e-9
        yield topic_frame_idx, timestamp_sec, points

        topic_frame_idx += 1
        yielded += 1


def _write_detection_csv(rows: list[dict], csv_path: Path) -> None:
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
        "point_max_distance_m",
        "point_count",
        "heading",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run rosbag PointCloud2 frames through OpenPCDet DSVT and pedestrian tracking"
    )
    parser.add_argument("bag", type=Path, help="rosbag2 directory or sqlite .db3 path")
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=10)
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
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/rosbag_dsvt_tracking"))
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = PipelineConfig(
        perception_name="openpcdet_dsvt",
        perception_score_threshold=args.score_threshold,
        perception_device=args.device,
        pedestrian_min_point_max_distance_m=args.pedestrian_min_point_max_distance,
        pedestrian_max_point_max_distance_m=args.pedestrian_max_point_max_distance,
    )
    pipeline = RealTimePedestrianTrackingPipeline(config)

    results: list[TrackingFrameResult] = []
    prediction_batches = []
    detection_rows: list[dict] = []

    for frame_id, timestamp_sec, points in _iter_lidar_frames(
        bag_path=args.bag,
        topic_name=args.topic,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
    ):
        frame = FrameInput(
            frame_id=frame_id,
            timestamp_sec=timestamp_sec,
            sensor_source=args.topic,
            payload={"points": points},
        )
        detections = pipeline.detector.infer(frame)
        pedestrian_detections = filter_pedestrians(
            detections,
            point_spread_filter=pipeline.pedestrian_point_spread_filter,
        )
        tracks = pipeline.tracker.update(
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
        results.append(result)
        prediction_batches.append(build_prediction_input(result))

        for detection in detections:
            item = asdict(detection)
            detection_rows.append(
                {
                    "frame": frame_id,
                    "timestamp_sec": timestamp_sec,
                    "class": item.pop("label"),
                    **item,
                }
            )

        print(
            f"frame={frame_id} points={len(points)} "
            f"detections={len(detections)} pedestrians={len(pedestrian_detections)} tracks={len(tracks)}"
        )

    if not results:
        raise SystemExit(f"No frames read from topic {args.topic!r} in {args.bag}")

    output_dir = args.output_dir
    write_tracking_csv(results, output_dir / "tracking_results.csv")
    write_tracking_json(results, output_dir / "tracking_results.json")
    write_prediction_input_json(prediction_batches, output_dir / "prediction_input.json")
    _write_detection_csv(detection_rows, output_dir / "detections.csv")
    print(f"saved outputs under: {output_dir}")


if __name__ == "__main__":
    main()
