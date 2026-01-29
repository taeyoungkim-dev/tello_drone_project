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
