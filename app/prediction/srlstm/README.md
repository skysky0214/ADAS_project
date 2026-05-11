# SR-LSTM 실시간 보행자 궤적 예측 시스템

LiDAR 기반 보행자 추적 데이터를 입력받아 **미래 궤적을 실시간으로 예측**하는 ADAS 시스템입니다.

## 모델 개요

| 항목 | 내용 |
|------|------|
| 베이스 모델 | SR-LSTM (States Refinement LSTM) |
| 관측 프레임 | **4프레임** (1.6초) |
| 예측 프레임 | **8프레임** (3.2초) |
| 데이터 FPS | 2.5Hz (0.4초/프레임) |
| 추론 속도 | 평균 **11ms** (CPU) |
| ADE / FDE | **0.395m / 0.737m** |
| 학습 데이터 | 캠퍼스 LiDAR 보행자 추적 데이터 (146명, 375프레임) |

### 슬라이딩 윈도우 방식

센서에서 매 프레임 좌표가 들어올 때마다 내부 버퍼에 저장하고, 4프레임이 모이면 자동으로 미래 8프레임을 예측합니다.

```
Frame 4 도착:  관측 [1,2,3,4]      → 예측 [5~12]  (8프레임, 3.2초 후까지)
Frame 5 도착:  관측 [2,3,4,5]      → 예측 [6~13]  ← 1번 버리고 5번 추가
Frame 6 도착:  관측 [3,4,5,6]      → 예측 [7~14]  ← 계속 밀림
...
```

항상 센서에서 들어온 **실제 데이터만** 사용합니다 (예측값 재사용 X → 오차 누적 방지).

---

## 파일 구조

```
app/prediction/srlstm/
├── realtime_predictor.py    # 실시간 예측기 (슬라이딩 윈도우 + TTC)
├── models.py                # SR-LSTM 모델 정의
├── basemodel.py             # GCN 등 기반 모듈
├── utils.py                 # 유틸리티
├── run_adas.py              # 센서 연동 실시간 루프 예시
├── visualize_realtime.py    # 궤적 시각화 GIF 생성
├── 전처리완료2.csv            # 테스트용 LiDAR 데이터
├── checkpoints/
│   └── E_obs4_pred8_59.tar  # 학습된 모델 가중치
└── README.md                # 이 문서
```

---

## 환경 설정

```bash
# Python 3.6+ 필요
pip install torch numpy pandas matplotlib
```

> GPU가 없어도 됩니다. CPU에서 프레임당 ~11ms로 충분히 실시간 동작합니다.

---

## 사용 방법

### 1. 기본 사용 (Python 코드 3줄)

```python
from realtime_predictor import RealtimePredictor, load_srlstm_model

# 모델 로드
model, args = load_srlstm_model('./checkpoints/E_obs4_pred8_59.tar')
predictor = RealtimePredictor(model, args, sensor_fps=2.5)

# 매 프레임 호출 (track_id: (x, y) 형태)
result = predictor.update(detections={
    1: (5.3, 2.1),   # 보행자 1번
    2: (-3.0, 7.5),  # 보행자 2번
})

# 결과 확인
for track_id, trajectory in result['predictions'].items():
    print(f"보행자 {track_id}: 3.2초 후 위치 = ({trajectory[-1, 0]:.1f}, {trajectory[-1, 1]:.1f})")
```

### 2. CSV 데이터로 테스트

```python
import pandas as pd
from realtime_predictor import RealtimePredictor, load_srlstm_model

model, args = load_srlstm_model('./checkpoints/E_obs4_pred8_59.tar')
predictor = RealtimePredictor(model, args, sensor_fps=2.5)

df = pd.read_csv('전처리완료2.csv')

for frame_idx in sorted(df.frame_index.unique()):
    frame_data = df[df.frame_index == frame_idx]
    detections = {int(row['track_id']): (row['x'], row['y']) 
                  for _, row in frame_data.iterrows()}
    
    result = predictor.update(detections=detections)
    
    if result['num_predicted'] > 0:
        print(f"Frame {frame_idx}: {result['num_predicted']}명 예측 완료")
```

### 3. 시각화 GIF 생성

```bash
python visualize_realtime.py
# → realtime_prediction.gif 생성
```

### 4. 실시간 ADAS 루프 (센서 연동)

`run_adas.py`의 `SensorInterface` 클래스를 실제 센서에 맞게 수정:

```python
class SensorInterface:
    def get_detections(self):
        # 여기에 실제 LiDAR/카메라 트래커 연동 코드 작성
        # 반환: {track_id: (x, y), ...}
        return detections
```

---

## 출력 형식

`predictor.update()` 반환값:

```python
{
    'predictions': {
        1: np.array([[x1,y1], [x2,y2], ..., [x8,y8]]),  # 8프레임 미래 좌표
        2: np.array([[x1,y1], ...]),
    },
    'ttc': {1: 2.4, 2: float('inf')},  # 충돌까지 남은 시간 (초)
    'alerts': [
        {'track_id': 1, 'ttc': 2.4, 'level': 'WARNING'}
    ],
    'num_tracked': 5,     # 추적 중인 보행자 수
    'num_predicted': 3,   # 예측 완료된 보행자 수
}
```

### 경고 등급

| 등급 | TTC | 의미 |
|------|:---:|------|
| `DANGER` | ≤ 1.5초 | 즉시 제동 필요 |
| `WARNING` | ≤ 3.0초 | 주의 필요 |
| `CAUTION` | > 3.0초 | 모니터링 |

---

## 성능

| 항목 | 수치 |
|------|:---:|
| ADE (평균 변위 오차) | 0.395 m |
| FDE (최종 변위 오차) | 0.737 m |
| 추론 속도 (CPU) | 평균 11ms, P95 14ms |
| 10Hz 예산(100ms) 초과율 | 0% |
| 버퍼 메모리 | ~1.6 KB (고정) |

### 10Hz LiDAR에서 사용 시

학습 데이터가 2.5Hz이므로, 10Hz 센서에서는 **4프레임마다 1개만** 사용하여 실효 FPS를 2.5Hz로 맞추세요. `sensor_fps=2.5`로 설정하면 TTC 등 시간 계산이 정확합니다.
