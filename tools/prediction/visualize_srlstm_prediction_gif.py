from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter


def make_prediction_gif(
    csv_path: Path,
    output_gif: Path,
    anchor_step: int = 1,
    max_anchors: int | None = None,
    min_tracks: int = 1,
    interval_ms: int = 220,
    selected_ids: list[int] | None = None,
) -> None:
    df = pd.read_csv(csv_path)
    required = {"frame", "anchor_frame", "step", "id", "x", "y", "type"}
    if not required.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {sorted(required)}")

    anchor_counts = df.groupby("anchor_frame")["id"].nunique().sort_index()
    anchors = [int(anchor) for anchor, count in anchor_counts.items() if int(count) >= min_tracks]
    anchors = anchors[:: max(anchor_step, 1)]
    if max_anchors is not None:
        anchors = anchors[:max_anchors]
    if not anchors:
        raise ValueError("No anchor frames available for GIF generation")

    color_map = plt.colormaps["tab20"]
    all_ids = sorted(df["id"].unique().tolist())
    if selected_ids:
        selected_id_set = set(selected_ids)
        df = df[df["id"].isin(selected_id_set)].copy()
        all_ids = [ped_id for ped_id in all_ids if ped_id in selected_id_set]

        anchor_counts = df.groupby("anchor_frame")["id"].nunique().sort_index()
        anchors = [int(anchor) for anchor, count in anchor_counts.items() if int(count) >= min_tracks]
        anchors = anchors[:: max(anchor_step, 1)]
        if max_anchors is not None:
            anchors = anchors[:max_anchors]
        if not anchors:
            raise ValueError("No anchor frames available after ID filtering")

    id_to_color = {ped_id: color_map(idx % color_map.N) for idx, ped_id in enumerate(all_ids)}

    x_min = float(df["x"].min()) - 1.0
    x_max = float(df["x"].max()) + 1.0
    y_min = float(df["y"].min()) - 1.0
    y_max = float(df["y"].max()) + 1.0

    fig, ax = plt.subplots(figsize=(9, 7))

    def update(frame_idx: int) -> None:
        anchor = anchors[frame_idx]
        frame_df = df[df["anchor_frame"] == anchor].copy()
        ids = sorted(frame_df["id"].unique().tolist())

        ax.clear()
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title(f"SR-LSTM Prediction Replay | anchor frame {anchor}")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")

        legend_added = {"gt_past": False, "gt_future": False, "pred": False}
        for ped_id in ids:
            ped_df = frame_df[frame_df["id"] == ped_id].copy()
            color = id_to_color[ped_id]

            past_df = ped_df[ped_df["type"] == "gt_past"].sort_values("frame")
            future_df = ped_df[ped_df["type"] == "gt_future"].sort_values("frame")
            pred_df = ped_df[ped_df["type"] == "pred"].sort_values("frame")

            if len(past_df):
                ax.plot(
                    past_df["x"],
                    past_df["y"],
                    color=color,
                    linewidth=2.0,
                    alpha=0.95,
                    label="Observed Past" if not legend_added["gt_past"] else None,
                )
                last = past_df.iloc[-1]
                ax.scatter([last["x"]], [last["y"]], color=[color], s=36)
                ax.text(last["x"] + 0.05, last["y"] + 0.05, f"ID {ped_id}", color=color, fontsize=8)
                legend_added["gt_past"] = True

            if len(future_df):
                ax.plot(
                    future_df["x"],
                    future_df["y"],
                    color=color,
                    linewidth=1.8,
                    linestyle="--",
                    alpha=0.7,
                    label="GT Future" if not legend_added["gt_future"] else None,
                )
                legend_added["gt_future"] = True

            if len(pred_df):
                ax.plot(
                    pred_df["x"],
                    pred_df["y"],
                    color=color,
                    linewidth=2.0,
                    linestyle="-.",
                    alpha=1.0,
                    label="Predicted Future" if not legend_added["pred"] else None,
                )
                legend_added["pred"] = True

        if any(legend_added.values()):
            ax.legend(loc="upper left")

    anim = FuncAnimation(fig, update, frames=len(anchors), interval=interval_ms, repeat=True)
    output_gif.parent.mkdir(parents=True, exist_ok=True)
    anim.save(output_gif, writer=PillowWriter(fps=max(1, int(1000 / interval_ms))))
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create SR-LSTM prediction replay GIF")
    parser.add_argument("prediction_csv", type=Path)
    parser.add_argument("output_gif", type=Path)
    parser.add_argument("--anchor-step", type=int, default=1)
    parser.add_argument("--max-anchors", type=int, default=None)
    parser.add_argument("--min-tracks", type=int, default=1)
    parser.add_argument("--interval-ms", type=int, default=220)
    parser.add_argument("--ids", type=int, nargs="*", default=None)
    args = parser.parse_args()

    make_prediction_gif(
        csv_path=args.prediction_csv,
        output_gif=args.output_gif,
        anchor_step=args.anchor_step,
        max_anchors=args.max_anchors,
        min_tracks=args.min_tracks,
        interval_ms=args.interval_ms,
        selected_ids=args.ids,
    )


if __name__ == "__main__":
    main()
