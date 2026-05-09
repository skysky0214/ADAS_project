from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


def load_tracking_results(json_path: Path) -> list[dict]:
    return json.loads(json_path.read_text(encoding="utf-8"))


def _collect_track_histories(frames: list[dict]) -> dict[int, list[tuple[float, float, int]]]:
    histories: dict[int, list[tuple[float, float, int]]] = {}
    for frame in frames:
        frame_id = int(frame["frame_id"])
        for track in frame["tracks"]:
            track_id = int(track["track_id"])
            histories.setdefault(track_id, []).append(
                (float(track["x"]), float(track["y"]), frame_id)
            )
    return histories


def replay_tracking_ids(
    json_path: Path,
    interval_ms: int = 250,
    tail: int = 10,
    save_gif: Path | None = None,
) -> None:
    frames = load_tracking_results(json_path)
    if not frames:
        raise ValueError("No tracking frames found")

    histories = _collect_track_histories(frames)
    colors = plt.cm.get_cmap("tab10", max(10, len(histories)))

    fig, ax = plt.subplots(figsize=(8, 6))

    xs = []
    ys = []
    for points in histories.values():
        xs.extend([p[0] for p in points])
        ys.extend([p[1] for p in points])
    pad = 1.0
    ax.set_xlim(min(xs) - pad, max(xs) + pad)
    ax.set_ylim(min(ys) - pad, max(ys) + pad)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Pedestrian Tracking ID Replay")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")

    def update(frame_idx: int):
        ax.clear()
        ax.set_xlim(min(xs) - pad, max(xs) + pad)
        ax.set_ylim(min(ys) - pad, max(ys) + pad)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title(f"Pedestrian Tracking ID Replay | frame {frames[frame_idx]['frame_id']}")
        ax.grid(True, alpha=0.3)
        ax.axis("equal")

        frame = frames[frame_idx]
        for track in frame["tracks"]:
            track_id = int(track["track_id"])
            color = colors(track_id % colors.N)
            x = float(track["x"])
            y = float(track["y"])
            history = track["history"][-tail:]
            hist_x = [float(p["x"]) for p in history]
            hist_y = [float(p["y"]) for p in history]

            ax.plot(hist_x, hist_y, color=color, linewidth=1.5, alpha=0.8)
            ax.scatter([x], [y], color=[color], s=60)
            ax.text(x + 0.05, y + 0.05, f"ID {track_id}", color=color, fontsize=10, weight="bold")

    anim = FuncAnimation(fig, update, frames=len(frames), interval=interval_ms, repeat=True)

    if save_gif is not None:
        save_gif.parent.mkdir(parents=True, exist_ok=True)
        anim.save(save_gif, writer=PillowWriter(fps=max(1, int(1000 / interval_ms))))
        plt.close(fig)
        return

    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay tracking IDs frame by frame")
    parser.add_argument("tracking_json", type=Path)
    parser.add_argument("--interval-ms", type=int, default=250)
    parser.add_argument("--tail", type=int, default=10)
    parser.add_argument("--save-gif", type=Path, default=None)
    args = parser.parse_args()

    replay_tracking_ids(
        json_path=args.tracking_json,
        interval_ms=args.interval_ms,
        tail=args.tail,
        save_gif=args.save_gif,
    )


if __name__ == "__main__":
    main()
