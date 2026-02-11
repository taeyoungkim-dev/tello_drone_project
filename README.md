# Tello Drone Face Tracking Project

> **DJI Tello 드론을 활용한 실시간 얼굴/사람 추적 시스템**  
> OpenCV와 YOLOv8을 이용한 자율 비행 제어 구현

---

## 프로젝트 개요

DJI Tello 드론에 **OpenCV Haar Cascade**와 **YOLOv8n**을 적용하여 실시간으로 얼굴 및 사람을 추적하는 자율 비행 시스템입니다. PID 제어를 통해 드론이 대상을 자동으로 추적하며, 성능 측정 및 영상 저장 기능을 포함합니다.

---

## 구현 기능

### 1. 객체 인식 및 추적
- **OpenCV Haar Cascade 기반 얼굴 추적** (`tello_face_opencv.py`)
  - 전통적인 컴퓨터 비전 기법 활용
  - 빠른 처리 속도, 하지만 조명과 각도에 민감
  
- **YOLOv8n 기반 사람 인식 및 추적** (`tello_face_yolov8.py`)
  - 딥러닝 기반 객체 검출
  - 다양한 환경에서 robust한 성능
  - `imgsz=320` 최적화로 추론 속도 2-3배 향상

### 2. 자율 비행 제어
- **PID 제어 시스템**
  - P-제어 (비례 제어) 중심 구현
  - Yaw (좌우 회전), Distance (전후), Up/Down (상하) 축 독립 제어
  - D-제어 (미분 제어) 준비 완료

### 3. 성능 최적화 시도
- **Producer-Consumer 패턴 적용** (`product_consumer_pattern_*.py`)
  - 영상 수신과 처리를 별도 스레드로 분리
  - 결과: 성능 개선 효과 미미 (Tello SDK 내부 최적화로 인해)
  
### 4. 성능 측정 및 분석
- **실시간 FPS 측정**: 화면에 Loop FPS 표시
- **비동기 로깅**: LogWriter 스레드로 메인 루프 영향 없이 CSV 저장
- **성능 분석 도구**: `analyze_performance.py`로 그래프 자동 생성

### 5. 영상 저장 기능
- **VideoWriter 스레드**: 비동기 비디오 저장
- **XVID 코덱** 사용 (연구용 표준)
- 처리된 프레임을 `videos/` 폴더에 자동 저장

### 6. 안전 기능
- **Pre-flight 모델 검증**: 이륙 전 AI 모델 로드 및 초기화 확인
- **에러 핸들링**: 모델 로드 실패 시 안전한 프로그램 종료
- **비상 착륙**: `q` 키 또는 KeyboardInterrupt 시 자동 착륙

---

## 실험 결과 및 주요 발견

### OpenCV vs YOLOv8n 비교
- **OpenCV Haar Cascade**: 얼굴 인식률이 낮음 (조명, 각도에 매우 민감)
- **YOLOv8n**: 안정적인 사람 인식 성능 제공 (다양한 환경에서 robust)
- **결론**: 실시간 추적에서는 YOLOv8n이 훨씬 우수한 성능

### Producer-Consumer 패턴의 한계
초기에는 영상 처리 지연을 해결하기 위해 **Producer-Consumer 패턴**을 적용했으나, 실제 측정 결과 **성능 개선 효과가 거의 없었습니다**.

**원인**: `djitellopy` 라이브러리 내부에서 이미 프레임 버퍼링과 최적화가 적용되어 있어, 외부에서 추가 스레딩을 적용해도 큰 효과가 없었습니다.

![성능 비교 그래프](image.png)

위 그래프에서 볼 수 있듯이, Producer-Consumer 패턴 적용 전후의 FPS 차이는 거의 없습니다. 이는 Tello SDK의 내부 최적화가 이미 충분히 효율적임을 의미합니다.

### 효과적이었던 최적화
1. **이미지 해상도 축소**: 320x240으로 축소하여 처리 속도 향상
2. **YOLO 입력 크기 최적화**: `imgsz=320`으로 추론 속도 2-3배 향상
3. **비동기 I/O**: LogWriter 및 VideoWriter 스레드로 메인 루프 보호

---

## 개발 환경

### Hardware
- **DJI Tello** (Ryze Tech)
  - 단안 카메라 (720p, 30fps)
  - WiFi 기반 영상 스트리밍

### Software
- **Python 3.x**
- **주요 라이브러리**:
  - `djitellopy` - Tello 드론 제어
  - `opencv-contrib-python` - 영상 처리
  - `ultralytics` - YOLOv8 객체 인식
  - `numpy` - 수치 연산
  - `pandas`, `matplotlib` - 성능 분석 및 시각화
  - `fpdf` - 캘리브레이션용 체스보드 생성

---

## 프로젝트 구조

```
tello_drone_project/
├── tello_face_opencv.py              # OpenCV 기반 얼굴 추적
├── tello_face_yolov8.py              # YOLOv8n 기반 사람 추적
├── product_consumer_pattern_opencv.py # Producer-Consumer 패턴 (OpenCV)
├── product_consumer_pattern_yolov8.py # Producer-Consumer 패턴 (YOLOv8)
├── analyze_performance.py             # 성능 분석 및 그래프 생성
├── calibration/
│   ├── generate_chessboard_pdf.py    # 캘리브레이션용 체스보드 생성
│   ├── calibration_capture.py        # 캘리브레이션 이미지 캡처
│   └── calibration_run.py            # 카메라 캘리브레이션 실행
├── logs/                             # 성능 측정 CSV 로그 저장
├── videos/                           # 처리된 영상 저장
└── requirements.txt                  # Python 패키지 의존성
```

---

## 사용 방법

### 1. 의존성 설치
```bash
pip install -r requirements.txt
```

### 2. Tello 드론 연결
- Tello 드론 전원 켜기
- WiFi에서 "TELLO-XXXXXX" 네트워크에 연결

### 3. 프로그램 실행
```bash
# OpenCV 버전
python tello_face_opencv.py

# YOLOv8 버전 (권장)
python tello_face_yolov8.py
```

### 4. 조작 방법
- 프로그램 실행 후 자동 이륙
- 드론이 자동으로 얼굴/사람 추적
- `q` 키를 누르면 착륙 및 종료

### 5. 성능 분석
```bash
python analyze_performance.py
```
- `logs/` 폴더의 CSV 파일을 분석하여 그래프 생성
- 결과는 `performance_comparison.png`로 저장

---

## 주요 학습 내용

1. **OpenCV vs YOLOv8**: 실시간 추적에서는 YOLOv8n이 더 robust한 성능 제공
2. **Producer-Consumer 패턴의 한계**: SDK 내부 최적화로 인해 추가 스레딩 효과 미미
3. **이미지 사이즈 최적화**: 해상도 축소가 가장 효과적인 최적화 방법
4. **비동기 I/O의 중요성**: 로깅/비디오 저장 등 I/O 작업은 별도 스레드 필수
5. **Pre-flight 검증**: 드론 이륙 전 모델 로드로 안전성 확보

---

## 향후 계획

현재 프로젝트에서 검증된 드론 제어 및 영상 처리 기술을 바탕으로, 다음과 같은 확장을 계획하고 있습니다:

### 카메라 캘리브레이션
- 체스보드 패턴을 활용한 카메라 내부 파라미터 추출
- 렌즈 왜곡 보정으로 정확한 3D 좌표 변환 가능

### ArUco 마커 기반 단안 비주얼 SLAM
- OpenCV의 `cv2.aruco` 모듈을 활용한 마커 검출
- `solvePnP`를 통한 3D 위치 및 자세 추정
- 실내 환경에서의 자율 주행 및 3D 공간 매핑

---

## 참고 자료

- [DJI Tello SDK Documentation](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
- [OpenCV Documentation](https://docs.opencv.org/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)
- [djitellopy GitHub](https://github.com/damiafuentes/DJITelloPy)

---

## License

This project is for educational and research purposes.

---

**Project by**: Taeyoung Kim  
**Last Updated**: 2026-02-02
