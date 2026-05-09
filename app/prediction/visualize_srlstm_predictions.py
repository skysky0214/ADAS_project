from __future__ import annotations

import sys
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_srlstm_predictions(
    csv_path: Path,
    output_path: Path,
    max_ids: int | None = None,
    selected_ids: list[int] | None = None,
) -> None:
    df = pd.read_csv(csv_path)
    required_cols = {"frame", "id", "x", "y", "type"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {sorted(required_cols)}")

    ids = sorted(df["id"].unique().tolist())
    if selected_ids:
        selected_id_set = set(selected_ids)
        ids = [ped_id for ped_id in ids if ped_id in selected_id_set]
    if max_ids is not None:
        ids = ids[:max_ids]
    if not ids:
        raise ValueError("No trajectory IDs found in prediction CSV")

    cols = 2
    rows = (len(ids) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(7 * cols, 5 * rows), squeeze=False)
    axes_flat = axes.flatten()

    for ax, ped_id in zip(axes_flat, ids):
        ped_df = df[df["id"] == ped_id].copy()

        past_df = ped_df[ped_df["type"] == "gt_past"].sort_values("frame")
        future_df = ped_df[ped_df["type"] == "gt_future"].sort_values("frame")
        pred_df = ped_df[ped_df["type"] == "pred"].sort_values("frame")

        if len(past_df):
            ax.plot(
                past_df["x"],
                past_df["y"],
                color="#1f77b4",
                marker="o",
                linewidth=2,
                label="GT Past",
            )
        if len(future_df):
            ax.plot(
                future_df["x"],
                future_df["y"],
                color="#2ca02c",
                marker="o",
                linewidth=2,
                linestyle="--",
                label="GT Future",
            )
        if len(pred_df):
            ax.plot(
                pred_df["x"],
                pred_df["y"],
                color="#d62728",
                marker="x",
                linewidth=2,
                linestyle="-.",
                label="Predicted Future",
            )

        if len(past_df):
            last_past = past_df.iloc[-1]
            ax.scatter(last_past["x"], last_past["y"], color="black", s=40, zorder=5)

        ax.set_title(f"Pedestrian ID {ped_id}")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")
        ax.legend()

    for ax in axes_flat[len(ids) :]:
        ax.axis("off")

    fig.suptitle("SR-LSTM Prediction Visualization", fontsize=16)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize SR-LSTM prediction CSV")
    parser.add_argument("prediction_csv", type=Path)
    parser.add_argument("output_png", type=Path)
    parser.add_argument("--max-ids", type=int, default=None)
    parser.add_argument("--ids", type=int, nargs="*", default=None)
    args = parser.parse_args()

    plot_srlstm_predictions(
        args.prediction_csv,
        args.output_png,
        max_ids=args.max_ids,
        selected_ids=args.ids,
    )


if __name__ == "__main__":
    main()
