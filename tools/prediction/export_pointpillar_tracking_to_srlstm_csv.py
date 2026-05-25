from __future__ import annotations

import argparse
import csv
from pathlib import Path


def export_pointpillar_tracking_csv(
    input_csv: Path,
    output_csv: Path,
    class_name: str = "Pedestrian",
) -> None:
    with input_csv.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        required = {"frame", "id", "class", "x", "y"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"CSV must contain columns: {sorted(required)}")

        rows = []
        for row in reader:
            if row["class"] != class_name:
                continue
            rows.append(
                {
                    "frame_index": int(row["frame"]),
                    "track_id": int(row["id"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                }
            )

    rows.sort(key=lambda item: (item["track_id"], item["frame_index"]))
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["frame_index", "track_id", "x", "y"])
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert pointpillar tracking CSV to SR-LSTM input CSV"
    )
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("output_csv", type=Path)
    parser.add_argument("--class-name", default="Pedestrian")
    args = parser.parse_args()

    export_pointpillar_tracking_csv(
        input_csv=args.input_csv,
        output_csv=args.output_csv,
        class_name=args.class_name,
    )


if __name__ == "__main__":
    main()
