from dataclasses import dataclass
from pathlib import Path


DEFAULT_OPENPCDET_ROOT = Path("/home/gh/workspaces/design_project/OpenPCDet")
DEFAULT_DSVT_CFG_FILE = (
    DEFAULT_OPENPCDET_ROOT
    / "tools/cfgs/custom_models/dsvt_pillar_sustech_ped_cyclist.yaml"
)
DEFAULT_DSVT_CHECKPOINT = (
    DEFAULT_OPENPCDET_ROOT
    / "output/cfgs/custom_models"
    / "dsvt_pillar_sustech_ped_cyclist/transfer_split_v1/ckpt/checkpoint_epoch_20.pth"
)
DEFAULT_SRLSTM_CHECKPOINT = (
    Path("/home/gh/workspaces/design_project/ADAS_project")
    / "app/prediction/srlstm/checkpoints/E_obs4_pred8_59.tar"
)


@dataclass(frozen=True)
class PipelineConfig:
    perception_name: str = "placeholder_detection_adapter"
    openpcdet_root: Path = DEFAULT_OPENPCDET_ROOT
    openpcdet_cfg_file: Path = DEFAULT_DSVT_CFG_FILE
    openpcdet_checkpoint: Path = DEFAULT_DSVT_CHECKPOINT
    perception_score_threshold: float = 0.1
    perception_device: str = "cuda"
    prediction_name: str = "none"
    srlstm_checkpoint: Path = DEFAULT_SRLSTM_CHECKPOINT
    srlstm_sensor_fps: float = 2.5
    tracker_match_distance: float = 1.2
    tracker_reconnect_distance: float = 2.4
    tracker_max_missed: int = 5
    history_size: int = 10
