from __future__ import annotations

import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parent.parent))

from app.config import PipelineConfig
from app.domain_types import FrameInput
from app.logging_utils import (
    write_prediction_input_json,
    write_tracking_csv,
    write_tracking_json,
)
from app.pipeline import RealTimePedestrianTrackingPipeline
from app.prediction.input_builder import build_prediction_input


def build_demo_frames() -> list[FrameInput]:
    frames: list[FrameInput] = []
    for frame_id in range(40):
        timestamp = round(frame_id * 0.1, 2)
        detections = []

        # Pedestrian 1 blinks out for a few frames and should still keep the same ID.
        if frame_id not in {8, 9, 10, 24}:
            detections.append(
                {
                    "label": "Pedestrian",
                    "score": 0.93,
                    "x": round(4.0 + (0.28 * frame_id), 3),
                    "y": round(1.0 + (0.09 * frame_id), 3),
                }
            )

        # Pedestrian 2 has a shorter missed streak.
        if frame_id not in {15, 16}:
            detections.append(
                {
                    "label": "Pedestrian",
                    "score": 0.9,
                    "x": round(7.2 - (0.18 * frame_id), 3),
                    "y": round(-1.5 + (0.22 * frame_id), 3),
                }
            )

        if frame_id % 5 == 0:
            detections.append(
                {
                    "label": "Cyclist",
                    "score": 0.72,
                    "x": round(8.0 + 0.1 * frame_id, 3),
                    "y": -2.0,
                }
            )
        if frame_id >= 18:
            detections.append(
                {
                    "label": "Pedestrian",
                    "score": 0.88,
                    "x": round(2.0 + 0.12 * (frame_id - 18), 3),
                    "y": round(-3.0 + 0.1 * (frame_id - 18), 3),
                }
            )

        frames.append(
            FrameInput(
                frame_id=frame_id,
                timestamp_sec=timestamp,
                sensor_source="mock_lidar",
                payload={"detections": detections},
            )
        )
    return frames


def main() -> None:
    config = PipelineConfig()
    pipeline = RealTimePedestrianTrackingPipeline(config)

    print("=== Real-Time Pedestrian Tracking Demo ===")
    print(f"perception adapter: {config.perception_name}")
    print(f"tracker match distance: {config.tracker_match_distance}")
    print(f"tracker reconnect distance: {config.tracker_reconnect_distance}")
    print(f"tracker max missed: {config.tracker_max_missed}")
    print()

    results = []
    prediction_batches = []
    for frame in build_demo_frames():
        result = pipeline.step(frame)
        prediction_input = build_prediction_input(result)
        results.append(result)
        prediction_batches.append(prediction_input)
        if frame.frame_id < 3 or frame.frame_id >= 37:
            print(f"[frame {frame.frame_id}] tracking result")
            print(pipeline.debug_dump(result))
            print(f"[frame {frame.frame_id}] prediction input")
            print(prediction_input)
            print()

    output_dir = Path(__file__).resolve().parent.parent / "artifacts"
    csv_path = output_dir / "tracking_results.csv"
    json_path = output_dir / "tracking_results.json"
    prediction_json_path = output_dir / "prediction_input.json"
    write_tracking_csv(results, csv_path)
    write_tracking_json(results, json_path)
    write_prediction_input_json(prediction_batches, prediction_json_path)
    print(f"saved csv: {csv_path}")
    print(f"saved json: {json_path}")
    print(f"saved prediction input json: {prediction_json_path}")


if __name__ == "__main__":
    main()
