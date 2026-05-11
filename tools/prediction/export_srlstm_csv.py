from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


def export_tracking_json_to_srlstm_csv(input_json: Path, output_csv: Path) -> None:
    payload = json.loads(input_json.read_text(encoding="utf-8"))
    rows = []

    for frame_item in payload:
        frame_id = int(frame_item["frame_id"])
        for track in frame_item["tracks"]:
            rows.append(
                {
                    "frame_index": frame_id,
                    "track_id": int(track["track_id"]),
                    "x": float(track["x"]),
                    "y": float(track["y"]),
                }
            )

    rows.sort(key=lambda item: (item["track_id"], item["frame_index"]))
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["frame_index", "track_id", "x", "y"])
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "Usage: python export_srlstm_csv.py <tracking_results.json> <output.csv>"
        )
    export_tracking_json_to_srlstm_csv(
        input_json=Path(sys.argv[1]),
        output_csv=Path(sys.argv[2]),
    )


if __name__ == "__main__":
    main()
