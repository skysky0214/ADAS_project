"""
실시간 SR-LSTM 추론 모듈 (ADAS용)
=================================
- 매 프레임 관측 좌표를 입력받아 슬라이딩 윈도우로 미래 궤적 예측
- 차량(ego vehicle) 위치 기준 TTC(Time-to-Collision) 계산
- 원본 SR-LSTM 모델을 그대로 로드하여 사용

Author: (based on SR-LSTM by Pu Zhang, CVPR 2019)
"""

import argparse
import ast
import time
import math
import numpy as np
import torch
import torch.nn as nn
from collections import defaultdict
from typing import Dict, List, Tuple, Optional


# ──────────────────────────────────────────────
# 모델 로딩 유틸리티
# ──────────────────────────────────────────────

def load_srlstm_model(checkpoint_path: str, args=None):
    """
    학습된 SR-LSTM 체크포인트를 로드하여 추론 모드로 반환.
    
    Args:
        checkpoint_path: .tar 체크포인트 파일 경로
        args: 모델 하이퍼파라미터 (None이면 기본값 사용)
    
    Returns:
        model: 로드된 SR-LSTM 모델 (eval 모드)
        args: 사용된 하이퍼파라미터
    """
    if args is None:
        args = get_default_args()

    # GPU 가용 여부에 따라 자동 설정
    args.using_cuda = torch.cuda.is_available()
    device = 'cuda:0' if args.using_cuda else 'cpu'

    from models import SR_LSTM
    model = SR_LSTM(args)

    checkpoint = torch.load(checkpoint_path, map_location=device)
    # strict=False: 체크포인트에 gcn1 등 일부 키가 없을 수 있음 (passing_time 차이)
    missing, unexpected = model.load_state_dict(checkpoint['state_dict'], strict=False)
    if missing:
        print(f"  [참고] 체크포인트에 없는 키: {len(missing)}개 (2차 SR Layer 미학습 시 정상)")
    model.eval()

    if args.using_cuda:
        model = model.cuda()

    print(f"[RealtimePredictor] 모델 로드 완료: {checkpoint_path}")
    print(f"  device={device}, obs_length={args.obs_length}, pred_length={args.pred_length}")
    return model, args


def get_default_args():
    """SR-LSTM 기본 하이퍼파라미터 반환 (train.py와 동일)"""
    args = argparse.Namespace(
        using_cuda=torch.cuda.is_available(),
        gpu=0,
        # 모델 구조
        input_size=2,
        output_size=2,
        input_embed_size=32,
        rnn_size=64,
        hidden_dot_size=32,
        ifdropout=True,
        dropratio=0.1,
        std_in=0.2,
        std_out=0.1,
        # States Refinement
        ifbias_gate=True,
        WAr_ac='',
        ifbias_WAr=False,
        rela_embed_size=32,
        rela_hidden_size=16,
        rela_layers=1,
        rela_input=2,
        rela_drop=0.1,
        rela_ac='relu',
        ifbias_rel=True,
        nei_hidden_size=64,
        nei_layers=1,
        nei_drop=0,
        nei_ac='',
        ifbias_nei=False,
        mp_ac='',
        nei_std=0.01,
        rela_std=0.3,
        WAq_std=0.05,
        passing_time=2,
        # 시퀀스 설정 (그리드 서치 최적값: obs=4/pred=8)
        seq_length=12,
        obs_length=4,
        pred_length=8,
        # 이웃
        neighbor_thred=10,
        grid_size=4,
        nei_thred_slstm=2,
        # 디버그
        ifdebug=False,
    )
    return args


# ──────────────────────────────────────────────
# 실시간 예측기
# ──────────────────────────────────────────────

class RealtimePredictor:
    """
    실시간 SR-LSTM 궤적 예측기.
    
    사용 흐름:
        1. 초기화 시 학습된 모델 로드
        2. 매 프레임 update()로 관측 좌표 입력
        3. obs_length 프레임이 모이면 자동으로 예측 실행
        4. 예측 결과 + TTC 반환
    
    좌표계:
        - 입력: 월드 좌표 (미터 단위 권장) — 예: LiDAR/카메라 → 월드 변환 후
        - 출력: 동일 좌표계의 미래 위치
    """

    def __init__(self, model, args, sensor_fps: float = 2.5):
        """
        Args:
            model: 로드된 SR-LSTM 모델
            args: 모델 하이퍼파라미터
            sensor_fps: 센서 프레임레이트 (Hz). 학습 데이터가 2.5Hz이므로
                        기본값 2.5. 10Hz LiDAR 사용 시 서브샘플링 필요.
                        예측 시간 간격 = 1/sensor_fps 초
        """
        self.model = model
        self.args = args
        self.sensor_fps = sensor_fps
        self.dt = 1.0 / sensor_fps  # 프레임 간 시간 간격 (초)

        self.obs_length = args.obs_length
        self.pred_length = args.pred_length
        self.seq_length = args.seq_length

        # 각 track_id별 관측 버퍼: {track_id: [(x, y), ...]}
        self.obs_buffer: Dict[int, List[Tuple[float, float]]] = defaultdict(list)

        # 마지막 예측 결과 캐시
        self.last_predictions: Dict[int, np.ndarray] = {}
        self.last_ttc: Dict[int, float] = {}

        self.frame_count = 0

    def update(self, detections: Dict[int, Tuple[float, float]],
               ego_position: Tuple[float, float] = (0.0, 0.0),
               ego_heading: float = 0.0,
               vehicle_width: float = 1.8,
               vehicle_length: float = 4.5,
               safety_margin: float = 1.0) -> dict:
        """
        매 프레임 호출. 관측 좌표를 입력받아 예측 + TTC 계산.
        
        Args:
            detections: {track_id: (x, y)} — 현재 프레임의 보행자/객체 좌표
            ego_position: 차량(자차) 현재 위치 (x, y)
            ego_heading: 차량 헤딩 (라디안, 0=동쪽, 반시계 양수)
            vehicle_width: 차량 폭 (미터)
            vehicle_length: 차량 길이 (미터)
            safety_margin: 안전 여유 거리 (미터)
        
        Returns:
            dict: {
                'frame': int,
                'predictions': {track_id: np.array shape (pred_length, 2)},
                'ttc': {track_id: float (초, inf=충돌 없음)},
                'alerts': [{'track_id': int, 'ttc': float, 'distance': float}],
                'inference_time_ms': float
            }
        """
        self.frame_count += 1

        # 1. 버퍼 업데이트
        active_ids = set(detections.keys())
        for track_id, (x, y) in detections.items():
            self.obs_buffer[track_id].append((x, y))
            # 버퍼 크기 제한 (obs_length만큼만 유지)
            if len(self.obs_buffer[track_id]) > self.obs_length:
                self.obs_buffer[track_id] = self.obs_buffer[track_id][-self.obs_length:]

        # 사라진 트랙 정리
        stale_ids = [tid for tid in self.obs_buffer if tid not in active_ids]
        for tid in stale_ids:
            del self.obs_buffer[tid]
            self.last_predictions.pop(tid, None)
            self.last_ttc.pop(tid, None)

        # 2. 예측 가능한 트랙 필터링 (obs_length 이상 관측된 것)
        ready_ids = [tid for tid, buf in self.obs_buffer.items()
                     if len(buf) >= self.obs_length]

        predictions = {}
        ttc_results = {}
        alerts = []

        if len(ready_ids) > 0:
            t_start = time.time()

            # 3. 모델 입력 텐서 구성
            inputs = self._build_input_tensor(ready_ids)

            # 4. 추론 실행
            with torch.no_grad():
                outputs, _, _, _ = self.model.forward(inputs, iftest=True)

            # 5. 예측 결과 추출
            pred_output = outputs.cpu().numpy()  # (seq_length-1, num_peds, 2)

            inference_time = (time.time() - t_start) * 1000  # ms

            for idx, track_id in enumerate(ready_ids):
                # obs_length-1 부터가 예측 구간 (autoregressive)
                pred_traj = pred_output[self.obs_length - 1:, idx, :]  # (pred_length, 2)

                # shift_value를 더해 절대 좌표로 복원
                last_obs = np.array(self.obs_buffer[track_id][-1])
                pred_traj_abs = pred_traj + last_obs

                predictions[track_id] = pred_traj_abs
                self.last_predictions[track_id] = pred_traj_abs

                # 6. TTC 계산
                ttc = self._compute_ttc(
                    pred_traj_abs, ego_position, ego_heading,
                    vehicle_width, vehicle_length, safety_margin
                )
                ttc_results[track_id] = ttc
                self.last_ttc[track_id] = ttc

                # 경고 생성
                current_dist = math.sqrt(
                    (last_obs[0] - ego_position[0]) ** 2 +
                    (last_obs[1] - ego_position[1]) ** 2
                )
                if ttc < float('inf'):
                    alerts.append({
                        'track_id': track_id,
                        'ttc': ttc,
                        'distance': current_dist,
                        'level': 'DANGER' if ttc < 1.5 else 'WARNING' if ttc < 3.0 else 'CAUTION'
                    })
        else:
            inference_time = 0.0

        # 경고를 TTC 기준 오름차순 정렬
        alerts.sort(key=lambda x: x['ttc'])

        return {
            'frame': self.frame_count,
            'predictions': predictions,
            'ttc': ttc_results,
            'alerts': alerts,
            'inference_time_ms': inference_time,
            'num_tracked': len(self.obs_buffer),
            'num_predicted': len(ready_ids),
        }

    def _build_input_tensor(self, track_ids: List[int]):
        """
        관측 버퍼로부터 SR-LSTM 입력 텐서를 구성.
        
        SR-LSTM forward()에 필요한 7개 입력:
            nodes_abs, nodes_norm, shift_value, seq_list, nei_list, nei_num, batch_pednum
        """
        num_peds = len(track_ids)
        seq_len = self.seq_length

        # 관측 데이터 배열 구성 (seq_length, num_peds, 2)
        # obs_length 이후는 0으로 채움 (모델이 autoregressive로 예측)
        nodes = np.zeros((seq_len, num_peds, 2))

        for idx, tid in enumerate(track_ids):
            obs = self.obs_buffer[tid][-self.obs_length:]
            for t, (x, y) in enumerate(obs):
                nodes[t, idx, 0] = x
                nodes[t, idx, 1] = y

        # seq_list: 데이터 존재 여부 (1=존재, 0=없음)
        seq_list = np.zeros((seq_len, num_peds))
        seq_list[:self.obs_length, :] = 1.0

        # shift_value: obs_length-1 시점 좌표로 정규화
        shift_point = nodes[self.obs_length - 1]  # (num_peds, 2)
        shift_value = np.repeat(shift_point.reshape(1, num_peds, 2), seq_len, axis=0)

        # nodes_norm: shift된 좌표
        nodes_norm = nodes - shift_value

        # nei_list: 이웃 관계 (obs 구간만 계산)
        nei_list = np.zeros((seq_len, num_peds, num_peds))
        nei_num = np.zeros((seq_len, num_peds))

        for t in range(self.obs_length):
            for i in range(num_peds):
                for j in range(num_peds):
                    if i == j:
                        continue
                    if seq_list[t, i] > 0 and seq_list[t, j] > 0:
                        dx = abs(nodes[t, i, 0] - nodes[t, j, 0])
                        dy = abs(nodes[t, i, 1] - nodes[t, j, 1])
                        if dx <= self.args.neighbor_thred and dy <= self.args.neighbor_thred:
                            nei_list[t, i, j] = 1.0
                            nei_num[t, i] += 1

        batch_pednum = [num_peds]

        # 텐서 변환
        def to_tensor(arr):
            t = torch.Tensor(arr)
            if self.args.using_cuda:
                t = t.cuda()
            return t

        # forward에는 seq_length-1 크기로 들어감
        inputs = (
            to_tensor(nodes[:seq_len - 1]),        # nodes_abs
            to_tensor(nodes_norm[:seq_len - 1]),    # nodes_norm
            to_tensor(shift_value[:seq_len - 1]),   # shift_value
            to_tensor(seq_list[:seq_len - 1]),       # seq_list
            to_tensor(nei_list[:seq_len - 1]),       # nei_list
            to_tensor(nei_num[:seq_len - 1]),        # nei_num
            to_tensor(np.array(batch_pednum * (seq_len - 1)).reshape(-1, 1)),  # batch_pednum
        )
        return inputs

    def _compute_ttc(self, pred_traj: np.ndarray,
                     ego_pos: Tuple[float, float],
                     ego_heading: float,
                     veh_w: float, veh_l: float,
                     margin: float) -> float:
        """
        예측 궤적과 차량 위치를 기반으로 TTC(Time to Collision) 계산.
        
        간단한 점-영역 교차 방식:
        - 차량을 중심으로 (length + margin) × (width + margin) 직사각형 영역
        - 예측 궤적의 각 시점이 이 영역에 진입하는 첫 번째 시점 = TTC
        
        Returns:
            TTC (초). 충돌 없으면 float('inf')
        """
        half_l = (veh_l + margin) / 2.0
        half_w = (veh_w + margin) / 2.0
        cos_h = math.cos(-ego_heading)
        sin_h = math.sin(-ego_heading)

        for t_idx in range(len(pred_traj)):
            # 차량 좌표계로 변환
            dx = pred_traj[t_idx, 0] - ego_pos[0]
            dy = pred_traj[t_idx, 1] - ego_pos[1]

            # 차량 헤딩 기준으로 회전
            local_x = dx * cos_h - dy * sin_h
            local_y = dx * sin_h + dy * cos_h

            # 직사각형 영역 내 진입 체크
            if abs(local_x) <= half_l and abs(local_y) <= half_w:
                # 시점 인덱스 → 실제 시간
                ttc = (t_idx + 1) * self.dt
                return ttc

        return float('inf')

    def get_prediction_times(self) -> np.ndarray:
        """예측 각 시점의 실제 시간(초) 배열 반환"""
        return np.arange(1, self.pred_length + 1) * self.dt

    def reset(self):
        """모든 버퍼 초기화"""
        self.obs_buffer.clear()
        self.last_predictions.clear()
        self.last_ttc.clear()
        self.frame_count = 0


# ──────────────────────────────────────────────
# 데모: 시뮬레이션 루프
# ──────────────────────────────────────────────

def demo_simulation():
    """
    시뮬레이션 데모.
    실제로는 LiDAR/카메라 트래커에서 detections를 받게 됩니다.
    """
    import sys
    sys.path.insert(0, '.')

    # 1. 모델 로드 (그리드 서치 최적 모델: obs=4, pred=8)
    checkpoint_path = './output/grid_search/0/E_obs4_pred8/E_obs4_pred8_59.tar'
    args = get_default_args()  # obs=4, pred=8, seq=12

    # ── 10Hz LiDAR에서 사용 시 ──
    # 학습 데이터가 2.5Hz이므로, 10Hz 센서에서는 4프레임마다 1개를 사용하여
    # 실효 FPS를 2.5Hz로 맞추는 것을 권장합니다.
    # sensor_fps=2.5로 설정하면 TTC 등 시간 계산이 올바르게 됩니다.

    model, args = load_srlstm_model(checkpoint_path, args)

    # 2. 실시간 예측기 생성 (2.5Hz = 0.4초/프레임)
    sensor_fps = 2.5
    predictor = RealtimePredictor(model, args, sensor_fps=sensor_fps)

    print("\n" + "=" * 60)
    print("  실시간 ADAS 예측 시뮬레이션 시작")
    print(f"  관측: {args.obs_length}프레임 ({args.obs_length / sensor_fps:.1f}초)")
    print(f"  예측: {args.pred_length}프레임 ({args.pred_length / sensor_fps:.1f}초)")
    print("=" * 60 + "\n")

    # 3. 가상 보행자 궤적 생성 (직진 + 곡선)
    total_frames = 50
    ego_pos = (0.0, 0.0)  # 차량은 원점에 정지해 있다고 가정

    for frame in range(total_frames):
        t = frame * 0.1  # 10Hz → 0.1초 간격

        # 보행자 1: 차량 쪽으로 직진
        ped1_x = 15.0 - t * 1.5  # 1.5 m/s로 접근
        ped1_y = 0.5

        # 보행자 2: 대각선 이동
        ped2_x = 10.0 - t * 0.8
        ped2_y = 5.0 - t * 1.0

        detections = {
            1: (ped1_x, ped1_y),
            2: (ped2_x, ped2_y),
        }

        # 4. 예측 실행
        result = predictor.update(
            detections=detections,
            ego_position=ego_pos,
            ego_heading=0.0,  # 동쪽 방향
            vehicle_width=1.8,
            vehicle_length=4.5,
            safety_margin=1.0,
        )

        # 5. 결과 출력
        if result['num_predicted'] > 0:
            print(f"[Frame {result['frame']:3d}] "
                  f"추적: {result['num_tracked']}명, "
                  f"예측: {result['num_predicted']}명, "
                  f"추론: {result['inference_time_ms']:.1f}ms")

            for alert in result['alerts']:
                level_icon = {'DANGER': '🔴', 'WARNING': '🟡', 'CAUTION': '🟢'}
                print(f"  {level_icon.get(alert['level'], '⚪')} "
                      f"ID {alert['track_id']}: "
                      f"TTC={alert['ttc']:.2f}초, "
                      f"거리={alert['distance']:.1f}m "
                      f"[{alert['level']}]")

            # 예측 궤적 상세 (첫 번째 트랙만)
            for tid, pred in result['predictions'].items():
                times = predictor.get_prediction_times()
                print(f"  → ID {tid} 예측 궤적:")
                for i in range(0, len(pred), 3):  # 3프레임 간격으로 출력
                    print(f"    t+{times[i]:.1f}s: ({pred[i, 0]:.2f}, {pred[i, 1]:.2f})")
                break

        print()


if __name__ == '__main__':
    demo_simulation()
