# Codex Handoff

## Purpose

This repository is the shared integration workspace for the team ADAS pipeline.

Current intended flow:

```text
ROS2 /lidar_points
-> OpenPCDet DSVT perception adapter
-> pedestrian filtering
-> tracking
-> SR-LSTM prediction
-> planner bridge
-> TTC warning candidate
-> RViz MarkerArray
```

## Repositories Around This Repo

This repo lives at:

- `/home/gh/workspaces/design_project/ADAS_project`

Related local repos/data currently used:

- `Pedestrian_GT`: `/home/gh/workspaces/design_project/Pedestrian_GT`
- `OpenPCDet`: `/home/gh/workspaces/design_project/OpenPCDet`
- `SUSTechPOINTS`: `/home/gh/workspaces/design_project/SUSTechPOINTS`
- `Cylinder3D`: `/home/gh/workspaces/design_project/Cylinder3D`
- `dataset`: `/home/gh/workspaces/design_project/dataset`

Important note:

- `Pedestrian_GT` was moved under `design_project`.
- Old path `/home/gh/workspaces/Pedestrian_GT` is currently a symlink for compatibility.

## Current Branch / Status

Repository state checked on May 10, 2026:

- branch: `develop`
- untracked: `.vscode/`

## What Works Right Now

These files define the currently working scaffold:

- [app/main.py](/home/gh/workspaces/design_project/ADAS_project/app/main.py:1)
- [app/pipeline/realtime_tracking.py](/home/gh/workspaces/design_project/ADAS_project/app/pipeline/realtime_tracking.py:1)
- [app/core/config.py](/home/gh/workspaces/design_project/ADAS_project/app/core/config.py:1)
- [app/core/domain_types.py](/home/gh/workspaces/design_project/ADAS_project/app/core/domain_types.py:1)
- [app/perception/base.py](/home/gh/workspaces/design_project/ADAS_project/app/perception/base.py:1)
- [app/perception/placeholder.py](/home/gh/workspaces/design_project/ADAS_project/app/perception/placeholder.py:1)
- [app/perception/pedestrian_filter.py](/home/gh/workspaces/design_project/ADAS_project/app/perception/pedestrian_filter.py:1)
- [app/tracking/pedestrian_tracker.py](/home/gh/workspaces/design_project/ADAS_project/app/tracking/pedestrian_tracker.py:1)
- [app/prediction/input_builder.py](/home/gh/workspaces/design_project/ADAS_project/app/prediction/input_builder.py:1)
- [app/core/logging_utils.py](/home/gh/workspaces/design_project/ADAS_project/app/core/logging_utils.py:1)

Verified behavior:

- `app/main.py` is now the ROS2 live pipeline entrypoint.
- It subscribes `/lidar_points`.
- It runs OpenPCDet DSVT, tracking, optional SR-LSTM prediction, planner snapshot, TTC warning, and RViz markers.
- It exports runtime artifacts under the selected output directory.

Generated artifacts:

- `artifacts/tracking_results.csv`
- `artifacts/tracking_results.json`
- `artifacts/prediction_input.json`

## What Still Needs Work

The core perception/tracking/prediction/planner snapshot imports now work.

Still incomplete:

- tracking ID stability is weak and affects prediction quality
- planner/control is only a JSON snapshot, not actual vehicle control
- no control command generation or vehicle interface exists yet

Verified imports:

- [app/bridge/planner_interface.py](/home/gh/workspaces/design_project/ADAS_project/app/bridge/planner_interface.py:1)
- [app/prediction/base.py](/home/gh/workspaces/design_project/ADAS_project/app/prediction/base.py:1)
- [app/prediction/placeholder.py](/home/gh/workspaces/design_project/ADAS_project/app/prediction/placeholder.py:1)
- [app/prediction/adapters/srlstm_predictor.py](/home/gh/workspaces/design_project/ADAS_project/app/prediction/adapters/srlstm_predictor.py:1)

## Practical Interpretation

This repo is currently a scaffold, not a finished end-to-end ADAS integration.

It already has:

- a simple perception adapter boundary
- an OpenPCDet DSVT-Pillar perception adapter for point cloud frames
- a pedestrian-only filter
- a working tracker
- a prediction input builder
- an SR-LSTM prediction adapter connected to `app/main.py`

It does not yet have:

- stable tracking performance
- real planner/control command generation
- a vehicle-facing control bridge

## Most Important Next Task

The best next integration step is:

1. run a longer ROS2 experiment on `rosbag2_2026_04_02-17_34_49`
2. inspect tracking ID stability and SR-LSTM prediction output
3. decide the planner/control command contract

Reason:

- DSVT, tracking, SR-LSTM, and planner snapshot are now connected
- prediction quality depends heavily on stable track IDs
- planner/control needs a concrete command interface before implementation

## Recommended Integration Strategy

### Option A: Fastest path

Connect a replay-style adapter first.

Example idea:

- read saved detection CSV / JSON from previous PointPillars or CenterPoint experiments
- convert each frame into `DetectedObject`
- feed it into the existing pipeline

This is the easiest way to prove the integration contract before doing real-time inference wiring.

### Option B: Better long-term path

Connect a real detector adapter.

Candidate sources:

- PointPillars output from OpenPCDet
- CenterPoint output
- DSVT output

This likely means:

- defining a detection frame contract
- building one adapter class under `app/perception`
- keeping `app/pipeline/realtime_tracking.py` mostly unchanged

## Relevant External Work Already Done In Pedestrian_GT

`Pedestrian_GT` already contains a lot of perception-side work:

- SUSTech GT generation and merge scripts
- OpenPCDet PointPillars training and evaluation
- MMDetection3D CenterPoint training and evaluation
- OpenPCDet DSVT experiments
- multiple evaluation and visualization scripts

Important local path:

- `/home/gh/workspaces/design_project/Pedestrian_GT`

Also important:

- path hardcoding there was recently cleaned up with a shared workspace helper
- key helper file:
  - [workspace_paths.py](/home/gh/workspaces/design_project/Pedestrian_GT/src/config/workspace_paths.py:1)

## Known Working / Useful Files In This Repo

For tracking and replay utilities:

- [tools/tracking/convert_pointpillar_csv_to_tracking_json.py](/home/gh/workspaces/design_project/ADAS_project/tools/tracking/convert_pointpillar_csv_to_tracking_json.py:1)
- [tools/prediction/export_pointpillar_tracking_to_srlstm_csv.py](/home/gh/workspaces/design_project/ADAS_project/tools/prediction/export_pointpillar_tracking_to_srlstm_csv.py:1)
- [tools/perception/visualize_detection_bev_gif.py](/home/gh/workspaces/design_project/ADAS_project/tools/perception/visualize_detection_bev_gif.py:1)
- [tools/perception/run_openpcdet_dsvt_frame.py](/home/gh/workspaces/design_project/ADAS_project/tools/perception/run_openpcdet_dsvt_frame.py:1)
- [app/main.py](/home/gh/workspaces/design_project/ADAS_project/app/main.py:1)

These are useful when doing a replay-first integration.

OpenPCDet DSVT smoke test:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python tools/perception/run_openpcdet_dsvt_frame.py \
  /home/gh/workspaces/design_project/OpenPCDet/data/sustech_ped_cyclist/points/000000.npy \
  --checkpoint /home/gh/workspaces/design_project/OpenPCDet/output/cfgs/custom_models/dsvt_pillar_sustech_ped_cyclist/transfer_split_v1/ckpt/checkpoint_epoch_20.pth \
  --output-json artifacts/dsvt_frame_000000.json \
  --output-csv artifacts/dsvt_frame_000000.csv
```

This was verified with `/home/gh/anaconda3/envs/pedestrian_gt/bin/python`.

ROS2 publish/subscribe experiment:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python app/main.py \
  --topic /lidar_points \
  --max-frames 10 \
  --score-threshold 0.1 \
  --output-dir artifacts/live_dsvt_tracking
```

In another terminal:

```bash
ros2 bag play /home/gh/workspaces/design_project/dataset/rosbag2_2026_04_02-17_34_49 --rate 0.2
```

This was verified for 1-2 frames. Outputs are written under the selected output directory.

ROS2 publish/subscribe with SR-LSTM prediction:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python app/main.py \
  --topic /lidar_points \
  --max-frames 20 \
  --score-threshold 0.1 \
  --prediction srlstm \
  --output-dir artifacts/live_dsvt_prediction
```

In another terminal:

```bash
ros2 bag play /home/gh/workspaces/design_project/dataset/rosbag2_2026_04_02-17_34_49 --rate 0.2
```

This was verified for 4 frames. SR-LSTM starts producing predictions after 4 observed frames.
Outputs include `predicted_trajectories.csv` and `predicted_trajectories.json`.

RViz2 live visualization is available from the same runtime node. Add `MarkerArray`
topic `/adas/tracking_markers` and set RViz2 `Fixed Frame` to `hesai_lidar` for
`rosbag2_2026_04_02-17_34_49`. Markers show tracked boxes, track IDs, history
lines, and SR-LSTM predicted paths. Use `--marker-frame <frame_id>` for another
bag frame, or `--no-rviz` to disable marker publishing.

TTC warning candidate generation is now in `app/control/ttc_warning.py`. The
runtime node writes `ttc_warnings.json`, and RViz2 warning
text is added for active L1/L2/L3 tracks. This is intentionally not a real vehicle
control command interface yet; it only records warning level and target
acceleration candidates until the hardware/control contract is agreed.

## Suggested Immediate Work Plan For The Next Codex Thread

1. Run a longer `ros2 bag play -> DSVT node -> tracking -> SR-LSTM` experiment on `rosbag2_2026_04_02-17_34_49`.
2. Tune score threshold / ROI before tracking if needed.
3. Inspect tracking ID stability from `tracking_results.csv`; prediction quality depends heavily on stable IDs.
4. Once perception/tracking/prediction behavior is acceptable, fix `bridge/planner_interface.py`.

## Suggested First Question To Reconfirm With The User

If needed, ask only this:

`Do you want the first integration to use replayed detector outputs from existing experiments, or direct inference from a live model path?`

## Quick Commands

Run current live node:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python app/main.py --topic /lidar_points --prediction srlstm
```

Quick import check for known broken modules:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
python - <<'PY'
try:
    import app.bridge.planner_interface as m
    print('planner ok', m)
except Exception as e:
    print('planner import failed:', type(e).__name__, e)
try:
    import app.prediction.placeholder as p
    print('prediction placeholder ok', p)
except Exception as e:
    print('prediction import failed:', type(e).__name__, e)
PY
```

## Bottom Line

Current state in one sentence:

`ADAS_project is a working tracking/prediction-input scaffold with placeholder perception, and the next real integration point should be app/perception using outputs or models from Pedestrian_GT.`
