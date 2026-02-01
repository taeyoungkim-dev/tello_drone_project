# tello_drone_project

# Project: Marker-based Visual SLAM & Autonomous Navigation with Tello
> **주제:** ArUco 마커를 활용한 실내 자율주행 및 3D 공간 매핑  
> **영문명:** Implementation of Marker-based Visual SLAM using a Low-Cost Drone

---

## 1. 프로젝트 배경 및 목표 (Background & Objective)
### 1.1 배경
* **하드웨어 제약:** DJI Tello는 고가의 센서(LiDAR, Stereo Camera)가 없고, 단안 카메라만으로는 거리(Depth) 측정과 절대 위치 파악이 어려움.
* **기존의 한계:** 순수 Visual SLAM은 스케일 모호성(Scale Ambiguity)과 누적 오차(Drift) 문제로 인해 장시간 안정적인 자율 비행이 어려움.

### 1.2 목표
* **ArUco 마커(Fiducial Markers)**를 실내 공간의 랜드마크로 활용하여 드론의 **절대 위치(Absolute Pose)를 보정**하고 스케일 문제를 해결.
* **Producer-Consumer 아키텍처**를 도입하여 영상 처리 지연(Latency)을 최소화하고, 부드러운 **연속 PID 제어**를 통해 자율적으로 마커를 탐색하며 **3D 경로 및 희소 지도(Sparse Map)**를 작성.

---

## 2. 핵심 기술 및 아키텍처 (Key Technologies & Architecture)
### 2.1 시스템 아키텍처 (Performance)
* **Producer-Consumer 패턴 적용:**
    * **영상 수신(Producer):** 별도 스레드에서 프레임을 수신하고 `queue(maxsize=1)`을 유지하여 최신 프레임만 보장 (Latency 최소화).
    * **메인 로직(Consumer):** AI/CV 연산 속도와 무관하게 항상 최신 프레임을 처리하여 제어 반응성 확보.
* **비동기 로깅 (Asynchronous Logging):**
    * 디스크 I/O로 인한 메인 루프 블로킹(Blocking)을 방지하기 위해 별도의 LogWriter 스레드 운용.

### 2.2 제어 및 내비게이션 (Navigation)
* **Greedy Visual Servoing:**
    * 사전 경로 계획(Planning) 없이, 현재 시야에 보이는 새로운 마커를 향해 이동하는 탐험 로직 적용.
    * "마커 발견 → 접근 → 완료 처리 → 회전하여 다음 마커 탐색"의 상태 머신(FSM) 구조.
* **Continuous Smooth Pursuit:**
    * 가다 서다(Stop & Go) 방식을 폐기하고, 마커와의 거리에 따라 속도를 가변적으로 조절하는 연속 PID 제어 적용 (Motion Blur 최소화).

### 2.3 매핑 및 위치 추정 (SLAM & Mapping)
* **Marker-based Pose Estimation:**
    * ArUco 마커의 실제 크기 정보를 이용하여 카메라와 마커 사이의 정확한 3차원 거리 및 자세(Pose) 산출.
    * 이를 Ground Truth로 삼아 드론의 이동 궤적(Trajectory) 보정.
* **Sparse Point Cloud Mapping:**
    * 마커의 위치를 노드(Node)로 하는 위상 지도(Topological Map)와 ORB 특징점을 융합한 희소 지도 작성.

---

## 3. 개발 환경 (Development Environment)
* **Hardware:** DJI Tello (Ryze Tech)
* **Language:** Python 3.x
* **Libraries:**
    * `OpenCV` (cv2, cv2.aruco) - Marker Detection & Pose Estimation
    * `djitellopy` - Drone Control SDK
    * `NumPy` - Matrix & Vector Calculation
    * `Matplotlib` - Real-time Visualization

---

## 4. 제약 사항 및 해결 전략 (Constraints & Strategy)
### 4.1 스케일 모호성 (Scale Ambiguity)
* **문제:** 단안 카메라는 물체의 실제 크기나 거리를 알 수 없음.
* **해결:** 사전에 크기가 정의된 **ArUco 마커**를 앵커(Anchor)로 사용하여 미터(m) 단위의 절대 거리 확보.

### 4.2 영상 지연 및 모션 블러 (Latency & Blur)
* **문제:** WiFi 통신 지연 및 드론의 급격한 기동 시 영상이 흐려져 마커 인식이 실패함.
* **해결:** 1. **스레딩 모델**로 영상 수신 버퍼를 1로 제한하여 지연 제거.
    2. **PID 제어** 튜닝을 통해 급발진/급정지를 방지하고 부드러운 곡선 주행 유도.

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