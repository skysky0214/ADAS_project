# Workspace File Map

## 목차

1. [개요](#개요)
2. [루트 문서](#루트-문서)
3. [런타임 앱 코드](#런타임-앱-코드)
4. [Core](#core)
5. [Pipeline](#pipeline)
6. [Perception](#perception)
7. [Tracking](#tracking)
8. [Prediction](#prediction)
9. [SR-LSTM 후보 모델](#sr-lstm-후보-모델)
10. [Planner Bridge](#planner-bridge)
11. [Control](#control)
12. [Tools](#tools)
13. [Docs](#docs)
14. [현재 주의할 점](#현재-주의할-점)

## 개요

이 문서는 현재 workspace 안의 주요 파일들이 어떤 역할을 하는지 빠르게 찾기 위한 지도다.

현재 동작하는 중심 흐름은 아래와 같다.

```text
ros2 bag play
-> /lidar_points
-> app/main.py
-> OpenPCDet DSVT perception
-> pedestrian filtering
-> pedestrian tracking
-> SR-LSTM prediction
-> TTC warning
-> RViz MarkerArray
-> artifact export
```

`app/main.py`가 현재 실제 ROS2 live pipeline 진입점이다. `tools/` 아래 파일은 보조 실행, 변환, 시각화 도구로 둔다.

## 루트 문서

- `README.md`
  - 프로젝트의 목적, 현재 데이터 계약, 실행 방법, 전체 구조를 설명한다.
  - 처음 보는 사람이 현재 scaffold를 이해할 때 보는 시작 문서다.

## 런타임 앱 코드

- `app/__init__.py`
  - `app`을 Python package로 인식시키는 파일이다.

- `app/main.py`
  - 현재 ROS2 live ADAS pipeline의 실행 진입점이다.
  - `/lidar_points`를 subscribe하고 DSVT 인지, tracking, SR-LSTM 예측, planner snapshot, TTC warning, RViz marker publish를 수행한다.
  - 실행 결과를 선택한 `artifacts/` 하위 output directory에 저장한다.

## Core

- `app/core/__init__.py`
  - core package의 설명용 초기화 파일이다.

- `app/core/config.py`
  - pipeline 설정값을 담는 `PipelineConfig`를 정의한다.
  - 현재는 tracker distance, max missed, history size 같은 작은 설정만 있다.

- `app/core/domain_types.py`
  - 현재 pipeline에서 공유하는 dataclass 타입을 정의한다.
  - 주요 타입은 `FrameInput`, `DetectedObject`, `PedestrianDetection`,
    `TrackedPedestrian`, `TrackingFrameResult`, `PredictionInputBatch`다.

- `app/core/logging_utils.py`
  - tracking 결과와 prediction input batch를 CSV/JSON으로 저장한다.
  - live pipeline artifact export에 사용한다.

## Pipeline

- `app/pipeline/__init__.py`
  - `RealTimePedestrianTrackingPipeline`을 package 레벨에서 바로 import할 수 있게 노출한다.

- `app/pipeline/realtime_tracking.py`
  - 실시간 프레임 처리 순서를 정의한다.
  - `PerceptionModel.infer()`를 호출하고, pedestrian만 필터링한 뒤 tracker에 넘긴다.
  - config에 따라 placeholder 또는 OpenPCDet DSVT adapter를 사용한다.

## Perception

- `app/perception/__init__.py`
  - perception package 초기화 파일이다.

- `app/perception/base.py`
  - 모든 perception adapter가 맞춰야 할 추상 경계인 `PerceptionModel`을 정의한다.
  - 입력은 `FrameInput`, 출력은 `list[DetectedObject]`다.

- `app/perception/placeholder.py`
  - 실제 detector가 붙기 전 사용하는 임시 perception adapter다.
  - `frame.payload["detections"]`에 들어온 mock detection dict를 `DetectedObject`로 변환한다.

- `app/perception/pedestrian_filter.py`
  - detection 결과 중 label이 `Pedestrian`인 항목만 골라 `PedestrianDetection`으로 변환한다.
  - tracking 단계가 보행자 전용 입력만 받도록 하는 얇은 필터다.

- `app/perception/adapters/__init__.py`
  - 실제 detector 출력 CSV/JSON replay, OpenPCDet/MMDetection3D inference 등을 연결할 위치다.

- `app/perception/adapters/openpcdet_dsvt.py`
  - OpenPCDet DSVT-Pillar 모델을 `PerceptionModel` 인터페이스로 감싸는 adapter다.
  - `frame.payload["points"]` 또는 `frame.payload["point_path"]`를 받아 `DetectedObject` 목록을 만든다.
  - 최종 인지 checkpoint `checkpoint_epoch_20.pth`를 config 기본값으로 사용한다.

- `app/perception/adapters/openpcdet_pointpillar.py`
  - OpenPCDet PointPillars 모델을 같은 `PerceptionModel` 인터페이스로 감싸는 adapter다.
  - latency 비교 시 `app/main.py --perception pointpillar`로 선택한다.

## Tracking

- `app/tracking/__init__.py`
  - tracking package 초기화 파일이다.

- `app/tracking/matcher.py`
  - detection과 track 예상 위치 사이의 유클리드 거리를 계산한다.
  - tracker의 matching 판단에 사용된다.

- `app/tracking/pedestrian_tracker.py`
  - 보행자 detection에 persistent track ID를 부여하고 유지한다.
  - missed frame 관리, 단순 속도 추정, history 누적을 담당한다.
  - 현재 working scaffold에서 가장 중요한 runtime module이다.

## Prediction

- `app/prediction/__init__.py`
  - prediction package 초기화 파일이다.

- `app/prediction/input_builder.py`
  - `TrackingFrameResult`를 `PredictionInputBatch`로 변환한다.
  - 각 track의 `history`를 prediction 모델 입력용 `observed_xy` sequence로 정리한다.

- `app/prediction/base.py`
  - trajectory prediction model의 추상 경계인 `PredictionModel`을 정의한다.
  - 입력은 `list[TrackedPedestrian]`, 출력은 `list[PredictedTrajectory]`다.

- `app/prediction/placeholder.py`
  - 실제 prediction 모델 전 단계의 선형 외삽 placeholder다.
  - 현재 track 속도 기반으로 미래 위치를 단순 extrapolation한다.

- `app/prediction/adapters/__init__.py`
  - SR-LSTM 등 모델별 input/output 변환 코드를 둘 위치다.

- `app/prediction/adapters/srlstm_predictor.py`
  - `TrackedPedestrian` 목록을 SR-LSTM realtime predictor 입력으로 변환한다.
  - 관측 4프레임이 쌓인 track에 대해 미래 8개 좌표를 `PredictedTrajectory`로 반환한다.

## SR-LSTM 후보 모델

- `app/prediction/srlstm/README.md`
  - SR-LSTM 실시간 예측 코드의 모델 개요, 사용법, 파일 구조를 설명한다.

- `app/prediction/srlstm/realtime_predictor.py`
  - SR-LSTM 모델을 로드하고, track별 sliding window buffer를 관리해 미래 궤적을 예측하는 코드다.

- `app/prediction/srlstm/models.py`
  - SR-LSTM 모델 구조를 정의한다.

- `app/prediction/srlstm/basemodel.py`
  - SR-LSTM 내부에서 사용하는 기반 neural network module을 정의한다.

- `app/prediction/srlstm/utils.py`
  - SR-LSTM 코드에서 사용하는 보조 함수 모음이다.

- `app/prediction/srlstm/run_adas.py`
  - 센서 연동 실시간 loop 예시 코드다.
  - 실제 ADAS sensor interface에 맞게 교체될 후보다.

- `app/prediction/srlstm/visualize_realtime.py`
  - SR-LSTM 실시간 예측 결과를 시각화하는 스크립트다.

- `app/prediction/srlstm/전처리완료2.csv`
  - SR-LSTM 테스트용 전처리 CSV 데이터다.

- `app/prediction/srlstm/checkpoints/E_obs4_pred8_59.tar`
  - 학습된 SR-LSTM checkpoint 파일이다.

## Planner Bridge

- `app/bridge/__init__.py`
  - bridge package 초기화 파일이다.

- `app/bridge/planner_interface.py`
  - tracking/prediction 결과를 planner-facing `PlannerSceneSnapshot`으로 바꾼다.
  - 입력은 `list[TrackedPedestrian]`와 `list[PredictedTrajectory]`다.

## Control

- `app/control/__init__.py`
  - control-facing adapter package 초기화 파일이다.

- `app/control/ttc_warning.py`
  - SR-LSTM 예측 궤적과 현재 tracking 결과로 TTC warning을 계산한다.
  - 실제 차량 제어 command는 만들지 않고, `level`, `action`, `target_accel_mps2` 후보만 만든다.
  - 기준값은 Level 1 `1.70s`, Level 2 `1.50s`, Level 3 `0.80s`다.

## Tools

### Perception Tools

- `tools/perception/visualize_detection_bev_gif.py`
  - rosbag2 LiDAR point cloud와 detection CSV를 BEV GIF로 시각화한다.
  - perception 결과를 눈으로 확인할 때 쓰는 오프라인 도구다.

- `tools/perception/run_openpcdet_dsvt_frame.py`
  - OpenPCDet DSVT-Pillar adapter를 단일 `.npy` 또는 `.bin` point cloud에 대해 실행한다.
  - checkpoint/config 로딩, CUDA inference, JSON/CSV export를 smoke test하는 도구다.

### Tracking Tools

- `tools/tracking/convert_pointpillar_csv_to_tracking_json.py`
  - PointPillars tracking CSV를 현재 tracking replay JSON 형식으로 변환한다.
  - replay-first integration을 할 때 detector 결과를 pipeline 입력에 맞추는 데 유용하다.

- `tools/tracking/visualize_tracking_ids.py`
  - tracking JSON을 frame별로 재생하며 track ID와 history tail을 시각화한다.
  - ID 유지가 잘 되는지 확인하는 도구다.

### Prediction Tools

- `tools/prediction/export_srlstm_csv.py`
  - 현재 `tracking_results.json`을 SR-LSTM 입력 CSV 형식으로 변환한다.
  - 출력 column은 `frame_index`, `track_id`, `x`, `y`다.

- `tools/prediction/export_pointpillar_tracking_to_srlstm_csv.py`
  - PointPillars tracking CSV에서 pedestrian만 골라 SR-LSTM 입력 CSV로 변환한다.

- `tools/prediction/visualize_srlstm_predictions.py`
  - SR-LSTM prediction CSV를 PNG trajectory plot으로 시각화한다.

- `tools/prediction/visualize_srlstm_prediction_gif.py`
  - SR-LSTM prediction CSV를 GIF로 시각화한다.

## Docs

- `docs/process.md`
  - 현재 설계 결정 과정과 data contract를 정리한다.
  - 왜 tracking history가 prediction 전에 필요한지 설명한다.

- `docs/CODEX_HANDOFF.md`
  - 다음 작업자가 현재 상태, 관련 repository, 작동/미작동 지점, 추천 다음 작업을 빠르게 이어받기 위한 handoff 문서다.

- `docs/file_map.md`
  - 이 문서다.
  - workspace 안 파일별 역할을 빠르게 찾기 위한 색인이다.

## 현재 주의할 점

- OpenPCDet DSVT adapter는 기본 Python이 아니라 `/home/gh/anaconda3/envs/pedestrian_gt/bin/python`에서 실행해야 한다.
- 실제 실험 환경 재현은 `app/main.py`를 먼저 띄우고 `ros2 bag play /home/gh/workspaces/design_project/dataset/rosbag2_2026_04_02-17_34_49`를 별도 터미널에서 실행한다.
- RViz2에서는 `Fixed Frame`을 bag의 LiDAR frame인 `hesai_lidar`로 두고, `/lidar_points`와 `/adas/tracking_markers`를 추가하면 된다.
- `app/prediction/base.py`, `app/prediction/placeholder.py`는 현행 `TrackedPedestrian` 타입 기준으로 import 가능하다.
- `app/bridge/planner_interface.py`는 planner-facing snapshot JSON 생성까지 가능하다.
- `app/prediction/srlstm/`은 `app/prediction/adapters/srlstm_predictor.py`를 통해 runtime node에 연결된다.
- `tools/` 아래 파일은 runtime package가 아니라 오프라인 변환/시각화 도구다.
