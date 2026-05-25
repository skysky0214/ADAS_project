# Design Process

## 1. Resetting the design

초기 스캐폴드는 perception 이후 prediction까지 한 번에 이어지는 형태로 작성되었다.  
하지만 실제로 prediction 모델에 필요한 입력을 먼저 생각하면, 그보다 앞 단계인 `Pedestrian Tracking`을 더 정확히 설계해야 했다.

즉, 먼저 필요한 질문은 다음이었다.

- 실시간으로 프레임이 들어오면 어떤 형태로 입력을 받을 것인가
- detection 결과에는 ID가 없는데, ID는 어느 단계에서 붙는가
- prediction 이전에 어떤 history를 누적해야 하는가

이 질문에 대한 답은 `ID별 x, y 시계열을 tracking 단계에서 만든다`는 것이었다.

## 2. Current scope

현재 범위는 prediction 이전까지다.

```text
frame input
-> object detection
-> pedestrian filtering
-> pedestrian tracking
-> id-wise x,y history
-> prediction input sequence
```

이 단계가 먼저 안정되어야 이후 prediction 모델 입력도 자연스럽게 정의할 수 있다.

## 3. Workspace layout

현재 workspace는 기능을 찾기 쉽게 아래처럼 나눈다.

- `app/core/`: config, domain types, output helpers
- `app/pipeline/`: demo frame source와 runtime pipeline 조립
- `app/perception/`: perception boundary, placeholder, pedestrian filtering, future adapters
- `app/tracking/`: pedestrian ID tracking
- `app/prediction/`: prediction input builder, placeholder model boundary, SR-LSTM 후보 코드
- `app/bridge/`: planner-facing interface 후보
- `tools/`: replay, conversion, export, visualization scripts

`perception`과 `prediction`은 아직 실제 모델 통합 전 단계이므로 `placeholder.py`,
`adapters/`, `srlstm/`처럼 역할이 드러나는 폴더를 둔다.

## 4. Design choice

현재 구조는 다음 원칙으로 잡았다.

1. 프레임 입력과 tracking 출력을 명확히 구분한다.
2. detection에는 ID를 넣지 않는다.
3. tracking이 ID를 부여하고 유지한다.
4. history는 tracking 내부 상태로 누적한다.
5. prediction은 이 history를 읽어 sequence input을 만든다.

## 5. Input / Output contract

### Frame input

- `frame_id`
- `timestamp_sec`
- `sensor_source`
- `payload["detections"]`

### Detection output

- `label`
- `score`
- `x`
- `y`

### Tracking output

- `track_id`
- `x`
- `y`
- `vx`
- `vy`
- `history`

여기서 가장 중요한 출력은 `history`이다.  
이 값이 prediction 모델 입력의 기초가 된다.

### Prediction input

- `track_id`
- `observed_xy`
- `current_xy`
- `velocity_xy`
- `history_len`

즉 tracking 출력 전체를 바로 모델에 넣는 것이 아니라,
prediction에 필요한 정보만 다시 정리한 sequence batch를 한 번 더 만든다.

## 6. What the experiment checks

현재 demo 실험은 아래를 검증한다.

- pedestrian만 잘 필터링되는지
- 같은 pedestrian에 같은 ID가 유지되는지
- 프레임이 진행될수록 x, y history가 누적되는지
- prediction input sequence가 기대한 형식으로 만들어지는지
- 결과가 CSV와 JSON으로 잘 저장되는지

## 7. Expected next step

다음 단계에서는 아래로 확장하면 된다.

1. 실제 detector 출력 연결
2. tracker 매칭 고도화
3. prediction input batch를 model input tensor 형식으로 변환
4. trajectory prediction 단계 추가
