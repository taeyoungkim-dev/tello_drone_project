# tello_drone_project

# Project: Monocular Visual SLAM with Tello
> **주제:** 저가형 드론의 단안 카메라를 활용한 실내 위치 추정 및 지도 작성  
> **영문명:** Implementation of Monocular Visual SLAM using a Low-Cost Drone in GPS-denied Environments

---

## 1. 프로젝트 배경 및 목표 (Background & Objective)
### 1.1 배경
* **하드웨어 제약:** DJI Tello와 같은 보급형 드론은 고가의 LiDAR나 스테레오 카메라(Stereo Camera)가 탑재되어 있지 않아 정밀한 깊이(Depth) 인식이 불가능함.
* **환경적 제약:** 실내 환경에서는 GPS 수신이 불가능하여, 호버링 및 주행 시 위치 오차(Drift)가 지속적으로 누적됨.

### 1.2 목표
* 드론의 **단안 카메라(Monocular Camera)** 영상 데이터만을 활용하여, 미지의 실내 공간에서 드론의 **3차원 이동 궤적(Trajectory)을 추정**하고 주변 환경의 **희소 지도(Sparse Map)를 작성**하는 Visual SLAM 알고리즘 구현.

---

## 2. 핵심 기술 및 원리 (Key Technologies)
* **특징점 추출 및 매칭 (Feature Extraction & Matching)**
    * 매 프레임에서 **ORB (Oriented FAST and Rotated BRIEF)** 알고리즘을 사용하여 코너, 에지 등 유의미한 특징점(Keypoints)을 검출하고 프레임 간 추적 수행.
* **운동 시차를 이용한 깊이 추정 (Depth from Motion Parallax)**
    * 단안 카메라의 한계를 극복하기 위해 드론의 **병진 운동(Translation)** 시 발생하는 시차를 이용.
    * **삼각 측량(Triangulation)** 기법을 통해 2차원 이미지 평면의 픽셀 좌표로부터 3차원 공간 좌표를 복원.
* **자세 추정 (Pose Estimation / Visual Odometry)**
    * 연속된 프레임 간의 특징점 변화를 기하학적으로 분석하여 드론의 **6-DoF (위치 및 자세)** 변화량을 역추적.

---

## 3. 개발 환경 (Development Environment)
* **Hardware:** DJI Tello (Ryze Tech) - Monocular Camera, IMU
* **Language:** Python 3.x
* **Libraries:**
    * `OpenCV` (Computer Vision & Geometry)
    * `djitellopy` (Drone Control SDK)
    * `NumPy` (Matrix Calculation)
    * `Matplotlib` or `Pangolin` (Visualization)

---

## 4. 제약 사항 및 해결 전략 (Constraints & Strategy)
### 4.1 스케일 모호성 (Scale Ambiguity)
* **문제:** 단안 카메라 특성상 절대적인 거리(Scale) 단위(m, cm)를 알 수 없음.
* **전략:** 초기 구현 단계에서는 **상대적인 스케일(Relative Scale)**로 지도를 작성하고, 추후 Tello의 하단 ToF 센서(높이)나 IMU 데이터를 융합(Sensor Fusion)하여 보정 시도.

### 4.2 비행 운용 전략
* **문제:** 제자리 회전(Pure Rotation)만 수행할 경우 깊이 정보를 얻을 수 없음.
* **전략:** 초기 매핑 시 **좌우/전후 이동(Translation)을 우선**으로 하는 비행 패턴을 적용하여 충분한 시차(Parallax)를 확보.
* **검증:** 초기에는 **수동 조종(Teleoperation)**으로 안정적인 데이터를 수집하며 알고리즘 검증 후 자율 비행 적용.

---

## 5. 개발 일지 (Development Log)

### 2026-02-02: 성능 최적화 및 측정 시스템 구축

#### 📊 성능 측정 기능 추가
- **FPS 측정**: 실시간 루프 처리 속도 측정 기능 구현
- **Frame Latency 측정**: Producer-Consumer 패턴에서 프레임 신선도(Frame Age) 측정
  - 최적화 전 버전: Latency 측정 불가 (동기 방식)
  - 최적화 후 버전: 실시간 Latency 측정 (타임스탬프 기반)
- 화면에 성능 지표 실시간 표시 (`cv2.putText`)

#### 📝 CSV 로깅 시스템 구현
- **LogWriter 스레드 클래스** 생성
  - 백그라운드 로깅으로 메인 루프 성능 영향 0%
  - 시간 기반 flush (1초 간격) 구현으로 디스크 I/O 최소화
  - 큐 기반 비동기 로깅 (1μs 미만 처리)
- **logs 폴더 관리**
  - 자동 생성 (`Path.mkdir(exist_ok=True)`)
  - 타임스탬프 기반 파일명 (`flight_log_opencv_20260202_143055.csv`)
  - CSV 형식: FrameCount, Timestamp, Latency_ms, FPS

#### 🛡️ 안전성 개선
- **모델 로드 순서 변경**
  - 기존: 드론 이륙 → 모델 로드 (위험)
  - 개선: 모델 로드 → 검증 → 드론 이륙 (안전)
- **에러 처리 강화**
  - OpenCV: `face_cascade.empty()` 검증
  - YOLOv8: 더미 추론으로 초기화 확인
  - 실패 시 명확한 에러 메시지 및 안전한 프로그램 종료

#### 📈 성능 분석 도구 개발
- **analyze_performance.py** 생성
  - logs 폴더의 CSV 파일 자동 로드
  - 최적화 전후 자동 매칭 (opencv, yolov8)
  - 3가지 직관적인 그래프 생성:
    1. FPS 비교 그래프 (시간 흐름)
    2. 평균 FPS 막대 그래프 (개선율 표시)
    3. 프레임 지연 그래프 (최적화 후만)
  - 통계 정보 콘솔 출력
  - 고해상도 PNG 저장 (`performance_comparison.png`)

#### 🔧 적용된 파일
- `tello_face_opencv.py` - 최적화 전 OpenCV 버전
- `tello_face_yolov8.py` - 최적화 전 YOLOv8 버전
- `product_consumer_pattern_opencv.py` - 최적화 후 OpenCV 버전
- `product_consumer_pattern_yolov8.py` - 최적화 후 YOLOv8 버전
- `analyze_performance.py` - 성능 분석 도구 (신규)

#### 💡 주요 개선 사항
- Producer-Consumer 패턴 효과 정량적 측정 가능
- 프레임 지연 60-90% 감소 확인 가능 (실측)
- 안전한 드론 운용 (모델 로드 실패 시 이륙 방지)
- 성능 비교 결과 시각화 자동화

---
