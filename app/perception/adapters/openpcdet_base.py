from __future__ import annotations

import os
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

import numpy as np

from app.core.domain_types import DetectedObject, FrameInput
from app.perception.base import PerceptionModel


CLASS_NAMES = ("Pedestrian", "Cyclist")


@contextmanager
def _temporary_cwd(path: Path) -> Iterator[None]:
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def _ensure_runtime_env() -> None:
    conda_prefix = os.environ.get("CONDA_PREFIX")
    if not conda_prefix:
        return
    include_dir = f"{conda_prefix}/targets/x86_64-linux/include"
    torch_lib_dir = f"{conda_prefix}/lib/python3.10/site-packages/torch/lib"
    os.environ.setdefault("CUDA_HOME", conda_prefix)
    os.environ["CPATH"] = f"{include_dir}:{os.environ.get('CPATH', '')}".rstrip(":")
    os.environ["LD_LIBRARY_PATH"] = (
        f"{conda_prefix}/lib:{torch_lib_dir}:{os.environ.get('LD_LIBRARY_PATH', '')}"
    ).rstrip(":")


def _load_points_from_payload(frame: FrameInput) -> np.ndarray:
    if "points" in frame.payload:
        points = np.asarray(frame.payload["points"], dtype=np.float32)
    elif "point_path" in frame.payload:
        point_path = Path(frame.payload["point_path"])
        if point_path.suffix == ".npy":
            points = np.load(point_path).astype(np.float32)
        elif point_path.suffix == ".bin":
            points = np.fromfile(point_path, dtype=np.float32).reshape(-1, 4)
        else:
            raise ValueError(f"Unsupported point cloud file extension: {point_path}")
    else:
        raise ValueError("OpenPCDet adapter expects frame.payload['points'] or ['point_path']")

    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Point cloud must have shape [N, >=3], got {points.shape}")
    if points.shape[1] == 3:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.concatenate([points, intensity], axis=1)
    if points.shape[1] > 4:
        points = points[:, :4]
    return np.ascontiguousarray(points, dtype=np.float32)


def _finite_positive(value: float) -> bool:
    return np.isfinite(value) and value > 0.0


def _points_in_box_footprint(
    points: np.ndarray,
    box: np.ndarray,
) -> np.ndarray | None:
    if points.size == 0:
        return points[:, :3]

    center_x, center_y = float(box[0]), float(box[1])
    dx, dy, heading = float(box[3]), float(box[4]), float(box[6])
    if not (_finite_positive(dx) and _finite_positive(dy) and np.isfinite(heading)):
        return None

    rel_x = points[:, 0] - center_x
    rel_y = points[:, 1] - center_y
    cos_h = np.cos(heading)
    sin_h = np.sin(heading)
    local_x = rel_x * cos_h + rel_y * sin_h
    local_y = -rel_x * sin_h + rel_y * cos_h

    mask = (np.abs(local_x) <= dx * 0.5) & (np.abs(local_y) <= dy * 0.5)
    return points[mask, :3]


def _max_pairwise_distance_m(points_xyz: np.ndarray) -> float | None:
    point_count = points_xyz.shape[0]
    if point_count == 0:
        return None
    if point_count == 1:
        return 0.0

    max_distance_sq = 0.0
    chunk_size = 512
    for start in range(0, point_count, chunk_size):
        chunk = points_xyz[start : start + chunk_size]
        diff = chunk[:, None, :] - points_xyz[None, :, :]
        distance_sq = np.sum(diff * diff, axis=2)
        max_distance_sq = max(max_distance_sq, float(np.max(distance_sq)))
    return float(np.sqrt(max_distance_sq))


def _point_spread_stats_in_box_footprint(
    points: np.ndarray,
    box: np.ndarray,
) -> tuple[float | None, int | None]:
    object_points = _points_in_box_footprint(points, box)
    if object_points is None:
        return None, None
    return _max_pairwise_distance_m(object_points), int(object_points.shape[0])


class OpenPCDetBasePerceptionModel(PerceptionModel):
    """Shared OpenPCDet adapter that converts point clouds into DetectedObject rows."""

    def __init__(
        self,
        openpcdet_root: Path,
        cfg_file: Path,
        checkpoint: Path,
        score_threshold: float = 0.1,
        device: str = "cuda",
    ):
        self.openpcdet_root = openpcdet_root.resolve()
        self.cfg_file = cfg_file.resolve()
        self.checkpoint = checkpoint.resolve()
        self.score_threshold = score_threshold
        self.device = device
        self._load_openpcdet_model()

    def infer(self, frame: FrameInput) -> list[DetectedObject]:
        points = _load_points_from_payload(frame)
        data_dict = {
            "points": points,
            "frame_id": frame.frame_id,
        }
        data_dict = self.dataset.prepare_data(data_dict=data_dict)
        batch_dict = self.dataset.collate_batch([data_dict])

        with self.torch.no_grad():
            self.load_data_to_gpu(batch_dict)
            pred_dicts, _ = self.model.forward(batch_dict)

        pred = pred_dicts[0]
        boxes = pred["pred_boxes"].detach().cpu().numpy()
        scores = pred["pred_scores"].detach().cpu().numpy()
        labels = pred["pred_labels"].detach().cpu().numpy()

        detections: list[DetectedObject] = []
        for box, score, label_idx in zip(boxes, scores, labels):
            score = float(score)
            if score < self.score_threshold:
                continue
            class_idx = int(label_idx) - 1
            if class_idx < 0 or class_idx >= len(self.class_names):
                continue
            point_max_distance_m, point_count = _point_spread_stats_in_box_footprint(points, box)
            detections.append(
                DetectedObject(
                    label=self.class_names[class_idx],
                    score=score,
                    x=float(box[0]),
                    y=float(box[1]),
                    z=float(box[2]),
                    dx=float(box[3]),
                    dy=float(box[4]),
                    dz=float(box[5]),
                    heading=float(box[6]),
                    point_max_distance_m=point_max_distance_m,
                    point_count=point_count,
                )
            )
        return detections

    def _load_openpcdet_model(self) -> None:
        if not self.openpcdet_root.exists():
            raise FileNotFoundError(f"OpenPCDet root not found: {self.openpcdet_root}")
        if not self.cfg_file.exists():
            raise FileNotFoundError(f"OpenPCDet config not found: {self.cfg_file}")
        if not self.checkpoint.exists():
            raise FileNotFoundError(f"OpenPCDet checkpoint not found: {self.checkpoint}")

        _ensure_runtime_env()
        if str(self.openpcdet_root) not in sys.path:
            sys.path.insert(0, str(self.openpcdet_root))
        tools_dir = self.openpcdet_root / "tools"
        if str(tools_dir) not in sys.path:
            sys.path.insert(0, str(tools_dir))

        import torch
        from pcdet.config import cfg, cfg_from_yaml_file
        from pcdet.datasets import DatasetTemplate
        from pcdet.models import build_network, load_data_to_gpu
        from pcdet.utils import common_utils

        self.torch = torch
        self.load_data_to_gpu = load_data_to_gpu

        with _temporary_cwd(self.openpcdet_root):
            cfg.clear()
            cfg.ROOT_DIR = self.openpcdet_root
            cfg.LOCAL_RANK = 0
            cfg_from_yaml_file(str(self.cfg_file), cfg)

        logger = common_utils.create_logger()

        class SingleFrameDataset(DatasetTemplate):
            def __init__(self, dataset_cfg, class_names, logger):
                super().__init__(
                    dataset_cfg=dataset_cfg,
                    class_names=class_names,
                    training=False,
                    root_path=None,
                    logger=logger,
                )

            def __len__(self):
                return 1

        self.class_names = list(getattr(cfg, "CLASS_NAMES", CLASS_NAMES))
        self.dataset = SingleFrameDataset(
            dataset_cfg=cfg.DATA_CONFIG,
            class_names=self.class_names,
            logger=logger,
        )
        self.model = build_network(
            model_cfg=cfg.MODEL,
            num_class=len(self.class_names),
            dataset=self.dataset,
        )

        requested_cuda = self.device.startswith("cuda")
        use_cuda = requested_cuda and torch.cuda.is_available()
        self.model.load_params_from_file(
            filename=str(self.checkpoint),
            logger=logger,
            to_cpu=not use_cuda,
        )
        if use_cuda:
            self.model.cuda()
        elif requested_cuda:
            raise RuntimeError("perception_device='cuda' was requested, but CUDA is not available")
        self.model.eval()
