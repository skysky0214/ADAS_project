from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import pandas as pd


def convert_pointpillar_csv(
    csv_path: Path,
    output_json: Path,
    class_name: str = "Pedestrian",
    max_frames: int | None = None,
) -> None:
    df = pd.read_csv(csv_path)
    required_cols = {"frame", "id", "class", "x", "y"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {sorted(required_cols)}")

    df = df[df["class"] == class_name].copy()
    df["frame"] = df["frame"].astype(int)
    df["id"] = df["id"].astype(int)
    df["x"] = df["x"].astype(float)
    df["y"] = df["y"].astype(float)
    df = df.sort_values(["frame", "id"]).copy()

    unique_frames = sorted(df["frame"].unique().tolist())
    if max_frames is not None:
        unique_frames = unique_frames[:max_frames]
        df = df[df["frame"].isin(unique_frames)].copy()

    track_histories: dict[int, list[dict]] = defaultdict(list)
    frames_payload: list[dict] = []

    for frame_id in unique_frames:
        frame_df = df[df["frame"] == frame_id]
        tracks = []
        for _, row in frame_df.iterrows():
            track_id = int(row["id"])
            point = {
                "frame_id": frame_id,
                "timestamp_sec": round(frame_id * 0.1, 3),
                "x": float(row["x"]),
                "y": float(row["y"]),
            }
            track_histories[track_id].append(point)

            history = track_histories[track_id][-10:]
            vx = 0.0
            vy = 0.0
            if len(history) >= 2:
                vx = history[-1]["x"] - history[-2]["x"]
                vy = history[-1]["y"] - history[-2]["y"]

            tracks.append(
                {
                    "track_id": track_id,
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "vx": vx,
                    "vy": vy,
                    "missed": 0,
                    "history": history,
                }
            )

        frames_payload.append(
            {
                "frame_id": frame_id,
                "timestamp_sec": round(frame_id * 0.1, 3),
                "detections": [],
                "tracks": tracks,
            }
        )

    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(frames_payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert pointpillar tracking CSV to tracking replay JSON"
    )
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_json", type=Path)
    parser.add_argument("--class-name", default="Pedestrian")
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args()

    convert_pointpillar_csv(
        csv_path=args.input_csv,
        output_json=args.output_json,
        class_name=args.class_name,
        max_frames=args.max_frames,
    )


if __name__ == "__main__":
    main()
