"""
실시간 SR-LSTM 예측 결과 시각화 (TTC 없이, 궤적만)
"""

import sys
sys.path.insert(0, '.')

import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import imageio
from realtime_predictor import RealtimePredictor, load_srlstm_model


def create_realtime_visualization(
    csv_path='전처리완료2.csv',
    checkpoint_path='./output/grid_search/0/E_obs4_pred8/E_obs4_pred8_59.tar',
    output_gif='realtime_prediction.gif',
    frame_step=2,
    fps_gif=8,
    trail_length=10,
):
    # 1. 모델 로드
    print('[1/4] Loading model...')
    model, args = load_srlstm_model(checkpoint_path)
    predictor = RealtimePredictor(model, args, sensor_fps=10.0)

    # 2. 데이터 로드
    print('[2/4] Loading CSV data...')
    df = pd.read_csv(csv_path)
    all_frames = sorted(df.frame_index.unique())

    # 색상
    colors = plt.cm.tab20(np.linspace(0, 1, 20))
    track_colors = {}
    for tid in df.track_id.unique():
        track_colors[int(tid)] = colors[int(tid) % 20]

    obs_trails = {}

    # 3. 프레임별 시각화
    print('[3/4] Generating frames...')
    images = []
    render_frames = all_frames[::frame_step]

    for fi, frame_idx in enumerate(render_frames):
        frame_data = df[df.frame_index == frame_idx]

        detections = {}
        for _, row in frame_data.iterrows():
            tid = int(row['track_id'])
            detections[tid] = (row['x'], row['y'])
            if tid not in obs_trails:
                obs_trails[tid] = []
            obs_trails[tid].append((row['x'], row['y']))
            if len(obs_trails[tid]) > trail_length:
                obs_trails[tid] = obs_trails[tid][-trail_length:]

        active = set(detections.keys())
        for tid in list(obs_trails.keys()):
            if tid not in active:
                del obs_trails[tid]

        # 예측 (TTC 계산 없이)
        result = predictor.update(detections=detections)

        # 그림
        fig, ax = plt.subplots(1, 1, figsize=(10, 8), facecolor='#1a1a2e')
        ax.set_facecolor('#16213e')
        ax.grid(True, alpha=0.15, color='#e2e2e2', linestyle='--', linewidth=0.5)
        ax.set_xlim(-22, 22)
        ax.set_ylim(-18, 18)
        ax.set_aspect('equal')

        for tid, (x, y) in detections.items():
            c = track_colors.get(tid, [0.5, 0.5, 0.5, 1.0])

            # 관측 궤적 꼬리
            if tid in obs_trails and len(obs_trails[tid]) > 1:
                trail = np.array(obs_trails[tid])
                alphas = np.linspace(0.1, 0.6, len(trail))
                for i in range(len(trail) - 1):
                    ax.plot(trail[i:i+2, 0], trail[i:i+2, 1],
                            color=c, alpha=float(alphas[i]), linewidth=1.5, zorder=3)

            # 현재 위치
            ax.scatter(x, y, s=80, c=[c], edgecolors='white',
                       linewidths=1.2, zorder=6, marker='o')
            ax.annotate(str(tid), (x, y), textcoords="offset points",
                        xytext=(5, 5), fontsize=7, color='white', fontweight='bold', zorder=7)

            # 예측 궤적
            if tid in result['predictions']:
                pred = result['predictions'][tid]
                pred_full = np.vstack([[x, y], pred])

                for i in range(len(pred_full) - 1):
                    alpha_val = 0.8 - (i / len(pred_full)) * 0.6
                    ax.plot(pred_full[i:i+2, 0], pred_full[i:i+2, 1],
                            color=c, alpha=alpha_val, linewidth=2,
                            linestyle='--', zorder=4)

                ax.scatter(pred[-1, 0], pred[-1, 1], s=50, c=[c],
                           marker='x', linewidths=2, alpha=0.8, zorder=5)

        # 제목 & 정보
        ax.set_title('SR-LSTM Real-time Prediction', fontsize=14,
                     fontweight='bold', color='#e2e2e2', pad=12)

        info_text = 'Frame %d/%d | Tracked: %d | Predicted: %d | Infer: %.0fms' % (
            frame_idx, all_frames[-1],
            result['num_tracked'], result['num_predicted'],
            result['inference_time_ms'])
        ax.text(0.5, -0.06, info_text, transform=ax.transAxes,
                ha='center', fontsize=10, color='#aaaaaa', fontfamily='monospace')

        # 범례
        legend_items = [
            plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='#4da6ff',
                       markersize=8, label='Current Position', linestyle='None'),
            plt.Line2D([0], [0], color='#4da6ff', linewidth=1.5, alpha=0.5,
                       label='Observed Trail'),
            plt.Line2D([0], [0], color='#4da6ff', linewidth=2, linestyle='--',
                       label='Predicted (12 steps)'),
            plt.Line2D([0], [0], marker='x', color='#4da6ff', markersize=8,
                       label='Pred End Point', linestyle='None'),
        ]
        leg = ax.legend(handles=legend_items, loc='upper left', fontsize=7,
                        facecolor='#16213e', edgecolor='#444444', framealpha=0.8)
        for text in leg.get_texts():
            text.set_color('#cccccc')

        ax.set_xlabel('X (m)', fontsize=9, color='#888888')
        ax.set_ylabel('Y (m)', fontsize=9, color='#888888')
        ax.tick_params(colors='#666666', labelsize=8)
        plt.tight_layout()

        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(h, w, 3)
        images.append(img)
        plt.close(fig)

        if (fi + 1) % 20 == 0 or fi == len(render_frames) - 1:
            print('  Frame %d/%d (%d/%d)' % (frame_idx, all_frames[-1], fi + 1, len(render_frames)))

    # 4. GIF 저장
    print('[4/4] Saving GIF (%d frames)...' % len(images))
    imageio.mimsave(output_gif, images, fps=fps_gif)
    file_size_mb = os.path.getsize(output_gif) / (1024 * 1024)
    print('Done! Saved: %s (%.1f MB)' % (output_gif, file_size_mb))


if __name__ == '__main__':
    create_realtime_visualization()
