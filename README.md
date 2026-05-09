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
perception_prediction_bridge/
  README.md
  docs/
    process.md
  app/
    main.py
    config.py
    domain_types.py
    logging_utils.py
    pipeline.py
    perception/
      base.py
      placeholder.py
      pedestrian_filter.py
    tracking/
      matcher.py
      pedestrian_tracker.py
    prediction/
      input_builder.py
```

## Module Roles

- `app/main.py`
  - 실시간 입력을 흉내 낸 demo frame 생성 및 실행
- `app/perception/placeholder.py`
  - detection 결과를 읽어 `DetectedObject`로 변환
- `app/perception/pedestrian_filter.py`
  - detection 중 `Pedestrian`만 추출
- `app/tracking/matcher.py`
  - detection과 기존 track의 거리 계산
- `app/tracking/pedestrian_tracker.py`
  - ID 부여, track 갱신, missed 관리, history 누적
- `app/domain_types.py`
  - 프레임, detection, tracked pedestrian, history 데이터 구조 정의
- `app/prediction/input_builder.py`
  - tracking 결과를 prediction 입력 시퀀스로 변환
- `app/logging_utils.py`
  - tracking 결과를 CSV/JSON으로 저장

## How to run

```bash
cd "/media/yeeun/새 볼륨/rosbag/perception_prediction_bridge"
python3 app/main.py
```

실행 후 결과는 아래에 저장된다.

```text
artifacts/tracking_results.csv
artifacts/tracking_results.json
artifacts/prediction_input.json
artifacts/srlstm_input.csv
```

## What is verified now

현재는 아래가 잘 기록되는지 확인할 수 있다.

- 프레임별 pedestrian detection
- 부여된 track ID
- 각 ID의 현재 x, y
- 추정된 vx, vy
- 누적된 x, y history
- prediction에 넣을 ID별 observed sequence
- SR-LSTM이 읽을 수 있는 `frame_index, track_id, x, y` CSV

## Next step

현재 JSON tracking 결과에서 SR-LSTM용 CSV를 만드는 스크립트:

- [export_srlstm_csv.py](/media/yeeun/새 볼륨/rosbag/perception_prediction_bridge/app/prediction/export_srlstm_csv.py)
- [visualize_srlstm_predictions.py](/media/yeeun/새 볼륨/rosbag/perception_prediction_bridge/app/prediction/visualize_srlstm_predictions.py)
- [visualize_tracking_ids.py](/media/yeeun/새 볼륨/rosbag/perception_prediction_bridge/app/tracking/visualize_tracking_ids.py)
- [convert_pointpillar_csv_to_tracking_json.py](/media/yeeun/새 볼륨/rosbag/perception_prediction_bridge/app/tracking/convert_pointpillar_csv_to_tracking_json.py)

다음 단계에서는 `PredictionInputBatch` 또는 `srlstm_input.csv`를 이용해 실제 SR-LSTM 추론으로 연결하면 된다.
