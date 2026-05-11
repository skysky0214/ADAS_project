from __future__ import annotations

import argparse
import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from PIL import Image
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def _box_corners_xy(x: float, y: float, dx: float, dy: float, heading: float) -> np.ndarray:
    half_x = dx / 2.0
    half_y = dy / 2.0
    corners = np.array(
        [
            [half_x, half_y],
            [half_x, -half_y],
            [-half_x, -half_y],
            [-half_x, half_y],
            [half_x, half_y],
        ],
        dtype=np.float32,
    )
    c = np.cos(heading)
    s = np.sin(heading)
    rot = np.array([[c, -s], [s, c]], dtype=np.float32)
    rotated = corners @ rot.T
    rotated[:, 0] += x
    rotated[:, 1] += y
    return rotated


def _load_points_by_frame(
    bag_db3: Path,
    topic_name: str,
    start_frame: int,
    max_frames: int,
) -> list[tuple[int, np.ndarray]]:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_db3), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    frame_idx = 0
    collected: list[tuple[int, np.ndarray]] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        if frame_idx < start_frame:
            frame_idx += 1
            continue
        if len(collected) >= max_frames:
            break

        msg = deserialize_message(data, PointCloud2)
        pts_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.asarray([(p[0], p[1], p[2]) for p in pts_iter], dtype=np.float32)
        collected.append((frame_idx, xyz))
        frame_idx += 1
    return collected


def create_detection_bev_gif(
    bag_db3: Path,
    detection_csv: Path,
    output_gif: Path,
    topic_name: str = "/lidar_points",
    start_frame: int = 0,
    max_frames: int = 60,
    point_stride: int = 3,
) -> None:
    det_df = pd.read_csv(detection_csv)
    det_df["frame"] = det_df["frame"].astype(int)

    frames = _load_points_by_frame(
        bag_db3=bag_db3,
        topic_name=topic_name,
        start_frame=start_frame,
        max_frames=max_frames,
    )
    if not frames:
        raise ValueError("No lidar frames loaded from bag")

    images: list[Image.Image] = []

    fig, ax = plt.subplots(figsize=(7, 7))
    for frame_id, xyz in frames:
        ax.clear()
        bev = xyz[:, :2]
        bev = bev[::point_stride]

        ax.scatter(bev[:, 0], bev[:, 1], s=0.5, c="#6a6a6a", alpha=0.65)
        frame_det = det_df[det_df["frame"] == frame_id]

        for _, row in frame_det.iterrows():
            corners = _box_corners_xy(
                x=float(row["x"]),
                y=float(row["y"]),
                dx=float(row["dx"]),
                dy=float(row["dy"]),
                heading=float(row["heading"]),
            )
            ax.plot(corners[:, 0], corners[:, 1], color="#d62728", linewidth=1.6)
            ax.text(
                float(row["x"]),
                float(row["y"]),
                f"{row['class']}",
                color="#d62728",
                fontsize=7,
            )

        ax.set_title(f"LiDAR + Detection Boxes | frame {frame_id}")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_xlim(-25, 25)
        ax.set_ylim(-25, 25)
        ax.grid(True, alpha=0.2)
        ax.axis("equal")

        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=140, bbox_inches="tight")
        buf.seek(0)
        images.append(Image.open(buf).convert("P"))

    plt.close(fig)
    output_gif.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(
        output_gif,
        save_all=True,
        append_images=images[1:],
        duration=120,
        loop=0,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Render LiDAR BEV + detection boxes to GIF")
    parser.add_argument("bag_db3", type=Path)
    parser.add_argument("detection_csv", type=Path)
    parser.add_argument("output_gif", type=Path)
    parser.add_argument("--topic", default="/lidar_points")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=60)
    parser.add_argument("--point-stride", type=int, default=3)
    args = parser.parse_args()

    create_detection_bev_gif(
        bag_db3=args.bag_db3,
        detection_csv=args.detection_csv,
        output_gif=args.output_gif,
        topic_name=args.topic,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
        point_stride=args.point_stride,
    )


if __name__ == "__main__":
    main()
