"""
실시간 센서 연동 예시 (ADAS)
============================
실제 차량에서 LiDAR/카메라 트래커 출력을 받아
매 프레임 예측하는 실사용 코드.

사용법:
    1. SensorInterface를 실제 센서에 맞게 구현
    2. python run_adas.py 실행
"""

import sys
import time
sys.path.insert(0, '.')

from realtime_predictor import RealtimePredictor, load_srlstm_model


# ─────────────────────────────────────────────────
# [1] 센서 인터페이스 — 이 부분만 실제 센서에 맞게 교체
# ─────────────────────────────────────────────────

class SensorInterface:
    """
    실제 센서 연결 시 이 클래스를 교체하세요.
    
    예시:
    - ROS: rospy.Subscriber로 /tracked_objects 토픽 구독
    - Socket: TCP/UDP로 트래커 서버에서 수신
    - 공유메모리: 트래커 프로세스와 메모리 공유
    """

    def __init__(self):
        """센서/트래커 초기화"""
        # === 실제 센서 연결 코드 ===
        # 예) self.ros_sub = rospy.Subscriber(...)
        # 예) self.socket = socket.connect(...)
        self.latest_detections = {}
        self.frame_count = 0
        print("[Sensor] Initialized (demo mode)")

    def get_detections(self):
        """
        현재 프레임의 트래킹 결과를 반환.
        
        Returns:
            dict: {track_id: (x, y)} — 미터 단위 월드 좌표
            
        실제 구현 시:
            - ROS: callback에서 받은 최신 데이터 반환
            - Socket: recv()로 데이터 수신 후 파싱
        """
        # === 여기를 실제 센서 코드로 교체 ===
        # 데모: 가상 보행자 생성
        self.frame_count += 1
        t = self.frame_count * 0.1  # 10Hz

        detections = {
            1: (12.0 - t * 1.0, 1.5),           # 정면에서 접근
            2: (8.0 - t * 0.5, 5.0 - t * 0.8),  # 대각선 이동
            3: (-5.0, 3.0),                       # 가만히 서있는 사람
        }
        return detections

    def get_ego_state(self):
        """
        차량(자차) 상태 반환.
        
        Returns:
            position: (x, y) 미터
            heading: 라디안 (0=동, 반시계=양)
            speed: m/s
        """
        # === 여기를 CAN 버스 / GPS / IMU 데이터로 교체 ===
        return (0.0, 0.0), 0.0, 0.0


# ─────────────────────────────────────────────────
# [2] 메인 루프 — 이 구조 그대로 사용
# ─────────────────────────────────────────────────

def main():
    # ── 모델 로드 (한 번만) ──
    print("=" * 55)
    print("  SR-LSTM ADAS Real-time Prediction System")
    print("=" * 55)

    checkpoint = './output/grid_search/0/E_obs4_pred8/E_obs4_pred8_59.tar'
    model, args = load_srlstm_model(checkpoint)
    predictor = RealtimePredictor(model, args, sensor_fps=2.5)

    # ── 센서 연결 ──
    sensor = SensorInterface()

    print("\n[System] Running at 10Hz... (Ctrl+C to stop)\n")

    # ── 실시간 루프 ──
    try:
        while True:
            loop_start = time.time()

            # 1) 센서에서 이번 프레임 데이터 받기
            detections = sensor.get_detections()
            ego_pos, ego_heading, ego_speed = sensor.get_ego_state()

            # 2) 예측 실행 (한 줄!)
            result = predictor.update(
                detections=detections,
                ego_position=ego_pos,
                ego_heading=ego_heading,
            )

            # 3) 결과 활용
            if result['num_predicted'] > 0:
                # 경고 출력
                for alert in result['alerts']:
                    if alert['level'] == 'DANGER':
                        print("[!!!] DANGER  ID %d  TTC=%.1fs  dist=%.1fm" % (
                            alert['track_id'], alert['ttc'], alert['distance']))
                    elif alert['level'] == 'WARNING':
                        print("[!!]  WARNING ID %d  TTC=%.1fs  dist=%.1fm" % (
                            alert['track_id'], alert['ttc'], alert['distance']))

                # 예측 좌표 출력 (5프레임마다)
                if result['frame'] % 5 == 0:
                    print("[F%3d] %d tracked, %d predicted, %.0fms" % (
                        result['frame'], result['num_tracked'],
                        result['num_predicted'], result['inference_time_ms']))
                    for tid, pred in result['predictions'].items():
                        print("  ID %d: +0.4s=(%.1f,%.1f) +0.8s=(%.1f,%.1f) +1.2s=(%.1f,%.1f)" % (
                            tid, pred[3,0], pred[3,1],
                            pred[7,0], pred[7,1],
                            pred[11,0], pred[11,1]))

            # 4) 2.5Hz 유지 (0.4초 간격)
            elapsed = time.time() - loop_start
            sleep_time = max(0, 0.4 - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[System] Stopped.")
        print("Total frames: %d" % predictor.frame_count)


if __name__ == '__main__':
    main()
