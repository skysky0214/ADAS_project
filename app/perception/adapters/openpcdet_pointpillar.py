from __future__ import annotations

from pathlib import Path

from app.perception.adapters.openpcdet_dsvt import OpenPCDetDSVTPerceptionModel


class OpenPCDetPointPillarPerceptionModel(OpenPCDetDSVTPerceptionModel):
    """OpenPCDet PointPillars adapter for SUSTech pedestrian/cyclist detection."""

    def __init__(
        self,
        openpcdet_root: Path,
        cfg_file: Path,
        checkpoint: Path,
        score_threshold: float = 0.1,
        device: str = "cuda",
    ):
        super().__init__(
            openpcdet_root=openpcdet_root,
            cfg_file=cfg_file,
            checkpoint=checkpoint,
            score_threshold=score_threshold,
            device=device,
        )
