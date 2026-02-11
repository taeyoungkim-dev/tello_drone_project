"""
성능 분석 및 시각화 도구
logs 폴더의 CSV 파일을 분석하여 최적화 전후 성능을 비교합니다.
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from pathlib import Path
import numpy as np

# 한글 폰트 설정 (한글이 깨지지 않도록)
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

def load_logs(log_dir="logs"):
    """logs 폴더에서 CSV 파일들을 로드"""
    log_path = Path(log_dir)
    
    if not log_path.exists():
        print(f"❌ {log_dir} 폴더가 없습니다!")
        return None
    
    csv_files = list(log_path.glob("*.csv"))
    
    if not csv_files:
        print(f"❌ {log_dir} 폴더에 CSV 파일이 없습니다!")
        return None
    
    print(f"📂 발견된 로그 파일: {len(csv_files)}개")
    for f in csv_files:
        print(f"   - {f.name}")
    
    logs = {}
    for file in csv_files:
        name = file.stem  # 확장자 없는 파일명
        try:
            df = pd.read_csv(file)
            # 시작 시간을 0으로 정규화
            df['Time_s'] = df['Timestamp'] - df['Timestamp'].iloc[0]
            logs[name] = df
            print(f"✅ {file.name} 로드 완료 ({len(df)}개 프레임)")
        except Exception as e:
            print(f"⚠️  {file.name} 로드 실패: {e}")
    
    return logs

def find_matching_pairs(logs):
    """최적화 전후 매칭되는 로그 쌍을 찾기"""
    pairs = []
    
    # OpenCV 쌍 찾기
    opencv_original = None
    opencv_optimized = None
    
    for name in logs.keys():
        if 'opencv' in name and 'optimized' not in name and 'yolo' not in name:
            opencv_original = name
        elif 'opencv_optimized' in name:
            opencv_optimized = name
    
    if opencv_original and opencv_optimized:
        pairs.append(('OpenCV', opencv_original, opencv_optimized))
    
    # YOLOv8 쌍 찾기
    yolo_original = None
    yolo_optimized = None
    
    for name in logs.keys():
        if 'yolov8' in name and 'optimized' not in name:
            yolo_original = name
        elif 'yolov8_optimized' in name:
            yolo_optimized = name
    
    if yolo_original and yolo_optimized:
        pairs.append(('YOLOv8', yolo_original, yolo_optimized))
    
    return pairs

def plot_performance_comparison(logs, pairs, save_path='performance_comparison.png'):
    """성능 비교 그래프 생성"""
    
    if not pairs:
        print("❌ 비교할 로그 쌍이 없습니다!")
        return
    
    # 서브플롯 개수 결정
    n_pairs = len(pairs)
    fig = plt.figure(figsize=(14, 5 * n_pairs))
    
    for idx, (name, original_key, optimized_key) in enumerate(pairs):
        df_old = logs[original_key]
        df_new = logs[optimized_key]
        
        # 통계 계산
        old_fps_mean = df_old['FPS'].mean()
        new_fps_mean = df_new['FPS'].mean()
        fps_improvement = ((new_fps_mean - old_fps_mean) / old_fps_mean) * 100
        old_latency_mean = df_old['Latency_ms'].mean()
        new_latency_mean = df_new['Latency_ms'].mean()
        
        # 서브플롯 생성 (1행 3열)
        base_idx = idx * 3 + 1
        
        # 1. FPS 비교 그래프
        ax1 = plt.subplot(n_pairs, 3, base_idx)
        ax1.plot(df_old['Time_s'], df_old['FPS'], 'r-', alpha=0.6, linewidth=1, label='Original')
        ax1.plot(df_new['Time_s'], df_new['FPS'], 'b-', alpha=0.6, linewidth=1, label='Optimized')
        ax1.axhline(old_fps_mean, color='red', linestyle='--', alpha=0.7, linewidth=2)
        ax1.axhline(new_fps_mean, color='blue', linestyle='--', alpha=0.7, linewidth=2)
        ax1.set_title(f'{name} - FPS Comparison', fontsize=12, fontweight='bold')
        ax1.set_xlabel('Time (seconds)', fontsize=10)
        ax1.set_ylabel('FPS', fontsize=10)
        ax1.legend(loc='lower right')
        ax1.grid(True, alpha=0.3)
        
        # 2. 평균 FPS 막대 그래프
        ax2 = plt.subplot(n_pairs, 3, base_idx + 1)
        bars = ax2.bar(['Original', 'Optimized'], [old_fps_mean, new_fps_mean], 
                       color=['#ff6b6b', '#4dabf7'], alpha=0.8, edgecolor='black', linewidth=2)
        ax2.set_title(f'{name} - Average FPS', fontsize=12, fontweight='bold')
        ax2.set_ylabel('FPS', fontsize=10)
        ax2.set_ylim(0, max(old_fps_mean, new_fps_mean) * 1.3)
        
        # 막대 위에 수치 표시
        for bar, value in zip(bars, [old_fps_mean, new_fps_mean]):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height,
                    f'{value:.1f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
        
        # 개선율 표시
        ax2.text(0.5, max(old_fps_mean, new_fps_mean) * 1.15,
                f'Improvement: +{fps_improvement:.1f}%',
                ha='center', fontsize=11, fontweight='bold', color='green',
                bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.3))
        ax2.grid(True, alpha=0.3, axis='y')
        
        # 3. Frame Latency 비교 (Original vs Optimized)
        ax3 = plt.subplot(n_pairs, 3, base_idx + 2)
        
        # Original Latency (대부분 0일 것임)
        ax3.plot(df_old['Time_s'], df_old['Latency_ms'], 'r-', alpha=0.6, linewidth=1, 
                label=f'Original (Avg: {old_latency_mean:.1f}ms)')
        
        # Optimized Latency
        ax3.plot(df_new['Time_s'], df_new['Latency_ms'], 'g-', alpha=0.7, linewidth=1.5, 
                label=f'Optimized (Avg: {new_latency_mean:.1f}ms)')
        
        # 평균선 표시
        if old_latency_mean > 0:
            ax3.axhline(old_latency_mean, color='red', linestyle='--', alpha=0.7, linewidth=2)
        ax3.axhline(new_latency_mean, color='darkgreen', linestyle='--', linewidth=2)
        
        ax3.set_title(f'{name} - Frame Latency Comparison', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Time (seconds)', fontsize=10)
        ax3.set_ylabel('Latency (ms)', fontsize=10)
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        # Original이 0이면 주석 추가
        if old_latency_mean < 1:
            ax3.text(0.5, 0.95, 'Note: Original has no latency measurement',
                    transform=ax3.transAxes, ha='center', va='top',
                    fontsize=9, style='italic', color='red',
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.7))
        
        # 통계 출력
        print(f"\n{'='*60}")
        print(f"{name} Performance Analysis")
        print(f"{'='*60}")
        print(f"Original Average FPS:      {old_fps_mean:>8.2f}")
        print(f"Optimized Average FPS:     {new_fps_mean:>8.2f}")
        print(f"FPS Improvement:           {fps_improvement:>7.1f}%")
        print(f"Original Frame Latency:    {old_latency_mean:>6.2f}ms")
        print(f"Optimized Frame Latency:   {new_latency_mean:>6.2f}ms")
        if old_latency_mean > 0:
            latency_reduction = ((old_latency_mean - new_latency_mean) / old_latency_mean) * 100
            print(f"Latency Reduction:         {latency_reduction:>7.1f}%")
        else:
            print(f"Latency Reduction:         N/A (Original not measured)")
        print(f"{'='*60}")
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n📊 그래프 저장 완료: {save_path}")
    plt.show()

def main():
    """메인 함수"""
    print("="*60)
    print("성능 분석 시작")
    print("="*60)
    
    # 로그 파일 로드
    logs = load_logs()
    
    if not logs:
        return
    
    # 매칭되는 쌍 찾기
    pairs = find_matching_pairs(logs)
    
    if not pairs:
        print("\n⚠️  최적화 전후 매칭되는 로그가 없습니다.")
        print("파일명에 'opencv', 'yolov8', 'optimized' 키워드가 포함되어야 합니다.")
        return
    
    print(f"\n✅ 발견된 비교 쌍: {len(pairs)}개")
    for name, orig, opt in pairs:
        print(f"   - {name}: {orig} vs {opt}")
    
    # 그래프 생성
    plot_performance_comparison(logs, pairs)
    
    print("\n✅ 분석 완료!")

if __name__ == "__main__":
    main()
