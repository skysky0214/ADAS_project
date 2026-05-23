## How to run: line159


# Real-Time Pedestrian Tracking Scaffold

이 코드베이스는 실시간 프레임 입력이 들어올 때, 객체 인지 후 보행자만 골라 `Pedestrian Tracking`으로 넘기고, 각 보행자에 대해 **ID별 x, y history**를 누적하는 구조를 먼저 명확히 잡기 위해 만들었다.

현재 단계의 핵심 목표는 prediction 이전 단계의 입력 형식을 고정하는 것이다.

```text
real-time frame
-> object detection
-> pedestrian filtering
-> pedestrian tracking
-> id-wise x,y history
-> prediction input sequence
```

## Why this stage comes first

trajectory prediction 모델은 단일 프레임 detection을 바로 받는 것이 아니라, 같은 보행자에 대해 시간 순서대로 누적된 좌표 history를 입력으로 받는다.

즉 먼저 필요한 것은 아래 두 가지다.

1. 현재 프레임에서 pedestrian만 분리하는 것
2. 같은 pedestrian을 프레임 간 같은 ID로 유지하는 것

이 코드베이스는 이 두 단계를 분리해서 보여준다.

## Current Data Contract

현재 프레임 입력:

```python
FrameInput(
    frame_id=12,
    timestamp_sec=1.2,
    sensor_source="mock_lidar",
    payload={
        "detections": [
            {"label": "Pedestrian", "score": 0.93, "x": 4.2, "y": 1.1}
        ]
    },
)
```

tracking 이후 출력:

```python
TrackingFrameResult(
    frame_id=12,
    timestamp_sec=1.2,
    detections=[...],
    tracks=[
        {
            "track_id": 1,
            "x": 4.2,
            "y": 1.1,
            "history": [...]
        }
    ],
)
```

즉 detection 단계에는 ID가 없고, tracking 단계에서 ID가 붙는다.

prediction 입력 형식:

```python
PredictionInputBatch(
    frame_id=12,
    timestamp_sec=1.2,
    sequences=[
        {
            "track_id": 1,
            "observed_xy": [(4.0, 1.0), (4.3, 1.1), (4.6, 1.2)],
            "current_xy": (4.6, 1.2),
            "velocity_xy": (0.3, 0.1),
            "history_len": 3,
        }
    ],
)
```

즉 prediction은 단일 detection이 아니라 `ID별 누적 x,y sequence`를 입력으로 받는다.

## Structure

```text
ADAS_project/
  README.md
  docs/
    process.md
  app/
    main.py
    core/
      config.py
      domain_types.py
      logging_utils.py
    pipeline/
      realtime_tracking.py
    perception/
      adapters/
        openpcdet_dsvt.py
      base.py
      placeholder.py
      pedestrian_filter.py
    tracking/
      matcher.py
      pedestrian_tracker.py
    prediction/
      adapters/
        srlstm_predictor.py
      srlstm/
      base.py
      input_builder.py
      placeholder.py
    bridge/
      planner_interface.py
  tools/
    perception/
    tracking/
    prediction/
```

## Module Roles

- `app/main.py`
  - ROS2 live ADAS pipeline 실행 진입점
- `app/core/config.py`
  - pipeline 설정값 정의
- `app/core/domain_types.py`
  - 프레임, detection, tracked pedestrian, history 데이터 구조 정의
- `app/core/logging_utils.py`
  - tracking 결과를 CSV/JSON으로 저장
- `app/pipeline/realtime_tracking.py`
  - perception, pedestrian filtering, tracking 실행 순서 정의
- `app/perception/placeholder.py`
  - detection 결과를 읽어 `DetectedObject`로 변환
- `app/perception/pedestrian_filter.py`
  - detection 중 `Pedestrian`만 추출
- `app/perception/adapters/`
  - 실제 detector 출력 또는 모델 inference 연결 위치
- `app/perception/adapters/openpcdet_dsvt.py`
  - OpenPCDet DSVT-Pillar checkpoint를 로드해 `DetectedObject`를 생성하는 perception adapter
- `app/tracking/matcher.py`
  - detection과 기존 track의 거리 계산
- `app/tracking/pedestrian_tracker.py`
  - ID 부여, track 갱신, missed 관리, history 누적
- `app/prediction/input_builder.py`
  - tracking 결과를 prediction 입력 시퀀스로 변환
- `app/prediction/adapters/srlstm_predictor.py`
  - tracking 결과의 `track_id -> (x, y)`를 SR-LSTM realtime predictor에 연결
- `app/prediction/srlstm/`
  - 실제 prediction 모델로 통합할 SR-LSTM 후보 코드
- `tools/`
  - 변환, export, 시각화 같은 오프라인 유틸리티

## How to run

OpenPCDet DSVT 단일 프레임 smoke test:

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python tools/perception/run_openpcdet_dsvt_frame.py \
  /home/gh/workspaces/design_project/OpenPCDet/data/sustech_ped_cyclist/points/000000.npy \
  --checkpoint /home/gh/workspaces/design_project/OpenPCDet/output/cfgs/custom_models/dsvt_pillar_sustech_ped_cyclist/transfer_split_v1/ckpt/checkpoint_epoch_20.pth \
  --output-json artifacts/dsvt_frame_000000.json \
  --output-csv artifacts/dsvt_frame_000000.csv
```

OpenPCDet adapter는 `torch`, `spconv`, CUDA extension이 필요하므로 현재 기본 Python이 아니라
`/home/gh/anaconda3/envs/pedestrian_gt/bin/python`에서 검증했다.

실제 ROS2 publish/subscribe 환경 재현:

터미널 1에서 ADAS node를 먼저 실행한다.

```bash
cd /home/gh/workspaces/design_project/ADAS_project
/home/gh/anaconda3/envs/pedestrian_gt/bin/python app/main.py \
  --topic /lidar_points \
  --score-threshold 0.1 \
  --latency-playback-rate 0.2 \
  --output-dir artifacts/live_dsvt_tracking
```

터미널 2에서 rosbag을 재생한다.

```bash
ros2 bag play /home/gh/workspaces/design_project/dataset/rosbag2_2026_04_02-17_34_49 --rate 0.2
```

짧은 확인만 할 때는 node에 `--max-frames 10`을 붙이면 해당 프레임 수 처리 후 자동 저장/종료한다.

SR-LSTM 예측까지 켜려면 node에 `--prediction srlstm`을 추가한다. SR-LSTM은 관측 4프레임이 쌓인 뒤부터 예측을 만든다.

```bash
/home/gh/anaconda3/envs/pedestrian_gt/bin/python app/main.py \
  --topic /lidar_points \
  --perception dsvt \
  --max-frames 20 \
  --score-threshold 0.1 \
  --prediction srlstm \
  --latency-playback-rate 0.2 \
  --ego-speed 10.0 \
  --safety-radius 1.0 \
  --output-dir artifacts/live_dsvt_prediction
```

예측 결과는 아래 파일로 저장된다.

```text
artifacts/live_dsvt_prediction/predicted_trajectories.csv
artifacts/live_dsvt_prediction/predicted_trajectories.json
artifacts/live_dsvt_prediction/ttc_warnings.json
artifacts/live_dsvt_prediction/latency.csv
```

`latency.csv`에는 callback 전체 처리 시간, point cloud 변환, perception, tracking, prediction, TTC warning, RViz marker publish 단계별 시간이 기록된다. rosbag 재생에서는 header stamp가 녹화 당시 시간이므로 wall-clock과 직접 비교하지 않고, `--latency-playback-rate` 기준 replay lag를 같이 기록한다.

PointPillars latency를 비교할 때는 `--perception pointpillar`로 바꿔 실행한다.

RViz2에서 현재 perception/tracking/prediction 결과를 같이 보려면 위 node를 그대로 실행한 뒤 RViz2에 아래 display를 추가한다.

```text
Fixed Frame: hesai_lidar
PointCloud2: /lidar_points
MarkerArray: /adas/tracking_markers
```

`/adas/tracking_markers`에는 파란 3D box, 흰색 track ID, 초록색 history, 빨간색 SR-LSTM prediction path, TTC warning text가 함께 publish된다. 다른 rosbag에서 frame id가 다르면 node 실행 시 `--marker-frame <frame_id>`로 고정할 수 있고, RViz publish가 필요 없으면 `--no-rviz`를 붙인다.

## What is verified now

현재는 아래가 잘 기록되는지 확인할 수 있다.

- 프레임별 pedestrian detection
- 부여된 track ID
- 각 ID의 현재 x, y
- 추정된 vx, vy
- 누적된 x, y history
- prediction에 넣을 ID별 observed sequence
- RViz2 MarkerArray 기반 box, ID, history, prediction path 표시
- TTC warning level과 감속도 후보 산출

## Next step

현재 JSON tracking 결과에서 SR-LSTM용 CSV를 만드는 스크립트:

- [export_srlstm_csv.py](/home/gh/workspaces/design_project/ADAS_project/tools/prediction/export_srlstm_csv.py)
- [visualize_srlstm_predictions.py](/home/gh/workspaces/design_project/ADAS_project/tools/prediction/visualize_srlstm_predictions.py)
- [visualize_tracking_ids.py](/home/gh/workspaces/design_project/ADAS_project/tools/tracking/visualize_tracking_ids.py)
- [convert_pointpillar_csv_to_tracking_json.py](/home/gh/workspaces/design_project/ADAS_project/tools/tracking/convert_pointpillar_csv_to_tracking_json.py)

다음 단계에서는 `PredictionInputBatch` 또는 `srlstm_input.csv`를 이용해 실제 SR-LSTM 추론으로 연결하면 된다.
