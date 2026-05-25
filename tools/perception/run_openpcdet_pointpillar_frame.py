from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import asdict
from pathlib import Path

if __package__ in (None, ""):
    sys.path.append(str(Path(__file__).resolve().parents[2]))

from app.core.config import (
    DEFAULT_OPENPCDET_ROOT,
    DEFAULT_POINTPILLAR_CFG_FILE,
    DEFAULT_POINTPILLAR_CHECKPOINT,
)
from app.core.domain_types import FrameInput
from app.perception.adapters.openpcdet_pointpillar import OpenPCDetPointPillarPerceptionModel


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run OpenPCDet PointPillars on one point cloud frame"
    )
    parser.add_argument("points", type=Path, help=".npy or .bin point cloud with x,y,z,intensity")
    parser.add_argument("--openpcdet-root", type=Path, default=DEFAULT_OPENPCDET_ROOT)
    parser.add_argument("--cfg-file", type=Path, default=DEFAULT_POINTPILLAR_CFG_FILE)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_POINTPILLAR_CHECKPOINT)
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--frame-id", type=int, default=0)
    parser.add_argument("--timestamp-sec", type=float, default=0.0)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--output-csv", type=Path, default=None)
    return parser


def write_csv(rows: list[dict], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["label", "score", "x", "y", "z", "dx", "dy", "dz", "heading"]
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = build_parser().parse_args()
    model = OpenPCDetPointPillarPerceptionModel(
        openpcdet_root=args.openpcdet_root,
        cfg_file=args.cfg_file,
        checkpoint=args.checkpoint,
        score_threshold=args.score_threshold,
        device=args.device,
    )
    frame = FrameInput(
        frame_id=args.frame_id,
        timestamp_sec=args.timestamp_sec,
        sensor_source="point_cloud_file",
        payload={"point_path": str(args.points)},
    )
    detections = model.infer(frame)
    rows = [asdict(item) for item in detections]

    print(f"detections: {len(rows)}")
    for row in rows[:20]:
        print(row)

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(rows, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    if args.output_csv is not None:
        write_csv(rows, args.output_csv)


if __name__ == "__main__":
    main()
