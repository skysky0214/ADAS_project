"""
ADAS Demo Data Generator
========================
OpenPCDet/CUDA 없이 UI 시연을 위한 데모 데이터를 생성합니다.
3단계 TTC 경고(보행자 확인/경고/위험)가 모두 나타나는 시나리오를 만듭니다.
"""
import json
import math
import os

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

NUM_FRAMES = 300  # 30초 (10Hz)
DT = 0.1  # 10 Hz

def generate_demo_data():
    tracking_results = []
    predicted_trajectories = []
    ttc_warnings = []
    latency_rows = []

    for i in range(NUM_FRAMES):
        t = i * DT
        frame_id = i
        timestamp_sec = 1000.0 + t

        tracks = []
        frame_predictions = []
        frame_warnings = []

        # === 보행자 1: 멀리서 접근 (TTC Level 1 → 2 → 3) ===
        ped1_start_x = 40.0
        ped1_approach_speed = 0.25  # m/frame (= 2.5 m/s)
        ped1_x = ped1_start_x - (i * ped1_approach_speed)
        ped1_y = 0.3 + 0.5 * math.sin(t * 0.3)  # 약간 좌우 흔들림

        if ped1_x > 2.0:
            ped1_history = []
            for h in range(min(i, 10)):
                hx = ped1_start_x - ((i - 10 + h) * ped1_approach_speed)
                hy = 0.3 + 0.5 * math.sin((t - (10 - h) * DT) * 0.3)
                ped1_history.append({"x": round(hx, 2), "y": round(hy, 2)})

            tracks.append({
                "track_id": 1,
                "x": round(ped1_x, 2),
                "y": round(ped1_y, 2),
                "history": ped1_history
            })

            # TTC 계산 (간단 근사)
            ttc = ped1_x / 10.0  # ego speed 10 m/s 기준
            if ttc > 3.0:
                level = 1
            elif ttc > 1.5:
                level = 2
            else:
                level = 3

            # 예측 경로 (앞으로 4초간)
            for step in range(1, 9):
                pred_t = step * 0.5
                pred_x = ped1_x - ped1_approach_speed * step * 5
                pred_y = ped1_y + 0.2 * math.sin(t * 0.3 + step * 0.5)
                frame_predictions.append({
                    "frame": frame_id,
                    "track_id": 1,
                    "predicted_x": round(pred_x, 2),
                    "predicted_y": round(pred_y, 2),
                    "predicted_t_sec": round(pred_t, 2)
                })

            if level == 1:
                label = "LEVEL_1_CAUTION"
                action = "warning_candidate"
                color = "level1"
                target_accel = 0.0
            elif level == 2:
                label = "LEVEL_2_WARNING"
                action = "s_curve_decel_candidate"
                color = "level2"
                target_accel = -3.0
            else:
                label = "LEVEL_3_EMERGENCY"
                action = "max_decel_candidate"
                color = "level3"
                target_accel = -8.0

            frame_warnings.append({
                "frame_id": frame_id,
                "track_id": 1,
                "level": level,
                "label": label,
                "action": action,
                "color": color,
                "min_ttc_sec": round(ttc, 2),
                "target_accel_mps2": target_accel
            })

        # === 보행자 2: 오른쪽에서 횡단 (중간에 나타남) ===
        if 50 <= i < 250:
            rel_i = i - 50
            ped2_x = 15.0 + 3.0 * math.sin(rel_i * 0.02)
            ped2_y = 5.0 - rel_i * 0.04  # 오른쪽에서 왼쪽으로 횡단

            ped2_history = []
            for h in range(min(rel_i, 10)):
                hrel = rel_i - 10 + h
                hx = 15.0 + 3.0 * math.sin(hrel * 0.02)
                hy = 5.0 - hrel * 0.04
                ped2_history.append({"x": round(hx, 2), "y": round(hy, 2)})

            tracks.append({
                "track_id": 2,
                "x": round(ped2_x, 2),
                "y": round(ped2_y, 2),
                "history": ped2_history
            })

            ttc2 = max(ped2_x / 10.0, 0.1)
            # 횡단 보행자는 가까울 때만 경고
            if abs(ped2_y) < 2.0:
                level2 = 2 if ttc2 > 1.0 else 3
            elif abs(ped2_y) < 4.0:
                level2 = 1
            else:
                level2 = 0

            for step in range(1, 7):
                pred_t = step * 0.5
                pred_x = ped2_x + 3.0 * math.sin((rel_i + step * 5) * 0.02) - ped2_x
                pred_y = ped2_y - step * 0.04 * 5
                frame_predictions.append({
                    "frame": frame_id,
                    "track_id": 2,
                    "predicted_x": round(ped2_x + pred_x * 0.3, 2),
                    "predicted_y": round(pred_y, 2),
                    "predicted_t_sec": round(pred_t, 2)
                })

            if level2 > 0:
                labels2 = {1: "LEVEL_1_CAUTION", 2: "LEVEL_2_WARNING", 3: "LEVEL_3_EMERGENCY"}
                actions2 = {1: "warning_candidate", 2: "s_curve_decel_candidate", 3: "max_decel_candidate"}
                colors2 = {1: "level1", 2: "level2", 3: "level3"}
                accels2 = {1: 0.0, 2: -3.0, 3: -8.0}
                frame_warnings.append({
                    "frame_id": frame_id,
                    "track_id": 2,
                    "level": level2,
                    "label": labels2[level2],
                    "action": actions2[level2],
                    "color": colors2[level2],
                    "min_ttc_sec": round(ttc2, 2),
                    "target_accel_mps2": accels2[level2]
                })

        # === 보행자 3: 먼 거리에서 걸어가는 안전한 보행자 ===
        if 30 <= i < 280:
            ped3_x = 35.0 + 5.0 * math.sin(i * 0.01)
            ped3_y = -6.0 + i * 0.01

            ped3_history = []
            for h in range(min(i - 30, 8)):
                hx = 35.0 + 5.0 * math.sin((i - 8 + h) * 0.01)
                hy = -6.0 + (i - 8 + h) * 0.01
                ped3_history.append({"x": round(hx, 2), "y": round(hy, 2)})

            tracks.append({
                "track_id": 3,
                "x": round(ped3_x, 2),
                "y": round(ped3_y, 2),
                "history": ped3_history
            })
            # 이 보행자는 멀어서 TTC 경고 없음 (level 0)

        # Assemble tracking frame
        tracking_results.append({
            "frame_id": frame_id,
            "timestamp_sec": round(timestamp_sec, 4),
            "tracks": tracks
        })

        predicted_trajectories.extend(frame_predictions)
        ttc_warnings.extend(frame_warnings)

        # Latency (시뮬레이션)
        latency_rows.append({
            "frame": frame_id,
            "perception_ms": round(12.0 + 3.0 * math.sin(i * 0.1), 1),
            "prediction_ms": round(5.0 + 2.0 * math.cos(i * 0.15), 1),
            "tracking_ms": round(1.5 + 0.5 * math.sin(i * 0.2), 1),
            "total_callback_ms": round(20.0 + 5.0 * math.sin(i * 0.08), 1),
        })

    # === Write files ===
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    with open(os.path.join(OUTPUT_DIR, "tracking_results.json"), "w") as f:
        json.dump(tracking_results, f, ensure_ascii=False)
    print(f"✅ tracking_results.json ({len(tracking_results)} frames)")

    with open(os.path.join(OUTPUT_DIR, "predicted_trajectories.json"), "w") as f:
        json.dump(predicted_trajectories, f, ensure_ascii=False)
    print(f"✅ predicted_trajectories.json ({len(predicted_trajectories)} predictions)")

    with open(os.path.join(OUTPUT_DIR, "ttc_warnings.json"), "w") as f:
        json.dump(ttc_warnings, f, ensure_ascii=False)
    print(f"✅ ttc_warnings.json ({len(ttc_warnings)} warnings)")

    # Latency CSV
    csv_path = os.path.join(OUTPUT_DIR, "latency.csv")
    with open(csv_path, "w") as f:
        headers = list(latency_rows[0].keys())
        f.write(",".join(headers) + "\n")
        for row in latency_rows:
            f.write(",".join(str(row[h]) for h in headers) + "\n")
    print(f"✅ latency.csv ({len(latency_rows)} rows)")

    print(f"\n🎉 데모 데이터 생성 완료! 경로: {OUTPUT_DIR}")
    print("   → 브라우저에서 http://localhost:8000 접속 후")
    print("   → 위 4개 파일을 하단 드래그 영역에 드래그 앤 드롭하세요!")


if __name__ == "__main__":
    generate_demo_data()
