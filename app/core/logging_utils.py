from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path

from app.core.domain_types import PredictionInputBatch, TrackingFrameResult


def write_tracking_csv(results: list[TrackingFrameResult], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "frame_id",
                "timestamp_sec",
                "track_id",
                "x",
                "y",
                "vx",
                "vy",
                "missed",
                "history_len",
                "history_xy",
            ]
        )
        for result in results:
            for track in result.tracks:
                writer.writerow(
                    [
                        result.frame_id,
                        result.timestamp_sec,
                        track.track_id,
                        round(track.x, 3),
                        round(track.y, 3),
                        round(track.vx, 3),
                        round(track.vy, 3),
                        track.missed,
                        len(track.history),
                        [(round(p.x, 3), round(p.y, 3)) for p in track.history],
                    ]
                )


def write_tracking_json(results: list[TrackingFrameResult], json_path: Path) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    payload = [asdict(item) for item in results]
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def write_prediction_input_json(
    batches: list[PredictionInputBatch], json_path: Path
) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    payload = [asdict(item) for item in batches]
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
