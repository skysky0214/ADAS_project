from dataclasses import dataclass
import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OPENPCDET_ROOT = Path(
    os.environ.get("OPENPCDET_ROOT", "/home/gh/workspaces/design_project/OpenPCDet")
)
DEFAULT_CONFIGS_ROOT = PROJECT_ROOT / "configs" / "openpcdet"
DEFAULT_PERCEPTION_CHECKPOINT_ROOT = PROJECT_ROOT / "app/perception/checkpoints"
DEFAULT_DSVT_CFG_FILE = (
    DEFAULT_CONFIGS_ROOT / "dsvt_pillar_sustech_ped_cyclist.yaml"
)
DEFAULT_DSVT_CHECKPOINT = (
    DEFAULT_PERCEPTION_CHECKPOINT_ROOT / "dsvt_checkpoint_epoch_20.pth"
)
DEFAULT_POINTPILLAR_CFG_FILE = (
    DEFAULT_CONFIGS_ROOT / "pointpillar_sustech_ped_cyclist.yaml"
)
DEFAULT_POINTPILLAR_CHECKPOINT = (
    DEFAULT_PERCEPTION_CHECKPOINT_ROOT / "pointpillar_checkpoint_epoch_20.pth"
)
DEFAULT_SRLSTM_CHECKPOINT = (
    PROJECT_ROOT / "app/prediction/srlstm/checkpoints/E_obs4_pred8_59.tar"
)


@dataclass(frozen=True)
class PipelineConfig:
    perception_name: str = "placeholder_detection_adapter"
    openpcdet_root: Path = DEFAULT_OPENPCDET_ROOT
    openpcdet_cfg_file: Path = DEFAULT_DSVT_CFG_FILE
    openpcdet_checkpoint: Path = DEFAULT_DSVT_CHECKPOINT
    pointpillar_cfg_file: Path = DEFAULT_POINTPILLAR_CFG_FILE
    pointpillar_checkpoint: Path = DEFAULT_POINTPILLAR_CHECKPOINT
    perception_score_threshold: float = 0.1
    perception_device: str = "cuda"
    prediction_name: str = "none"
    srlstm_checkpoint: Path = DEFAULT_SRLSTM_CHECKPOINT
    srlstm_sensor_fps: float = 2.5
    tracker_match_distance: float = 1.2
    tracker_reconnect_distance: float = 2.4
    tracker_max_missed: int = 5
    history_size: int = 10
