import cv2
import numpy as np
from djitellopy import Tello

# ==========================================
# 1. 설정 및 초기화
# ==========================================
print("드론 연결 중...")
me = Tello()
me.connect()
print(f"배터리: {me.get_battery()}%")

me.streamon()

# ------------------------------------------
# [중요] 카메라 캘리브레이션 데이터 (가정값)
# ------------------------------------------
# 카메라의 초점거리(focal length)와 중심점(optical center)을 알아야
# 2D 화면의 픽셀을 3D 공간의 좌표로 바꿀 수 있습니다.
# Tello의 정확한 값은 캘리브레이션을 해야 하지만, 여기선 근사값을 씁니다.
w, h = 360, 240  # 연산 속도를 위해 이미지 크기를 줄일 예정
focal_length = 300 # 대략적인 초점 거리
pp = (w//2, h//2)  # 화면의 중심점 (Principal Point)

# 카메라 매트릭스 (Intrinsic Matrix) 구성
K = np.array([[focal_length, 0, pp[0]],
              [0, focal_length, pp[1]],
              [0, 0, 1]])

# ------------------------------------------
# 변수 초기화
# ------------------------------------------
# 드론의 현재 위치 (X, Y, Z) - 시작점은 (0, 0, 0)
curr_R = np.eye(3) # 회전 행렬 (처음엔 회전 없음)
curr_t = np.zeros((3, 1)) # 이동 벡터 (처음엔 이동 없음)

# 궤적을 그릴 빈 지도 (검은색 배경)
traj_map = np.zeros((600, 600, 3), dtype=np.uint8)

# ORB 특징점 추출기 생성
orb = cv2.ORB_create(nfeatures=1000)

print("초기화 완료. 잠시 후 시작합니다...")
# 첫 번째 프레임을 읽어서 '이전 프레임(prev)'으로 저장
prev_frame = me.get_frame_read().frame
prev_frame = cv2.resize(prev_frame, (w, h))
# 첫 번째 프레임에서 특징점(kp)과 기술자(des) 추출
prev_kp, prev_des = orb.detectAndCompute(prev_frame, None)


while True:
    # ==========================================
    # 2. 새로운 프레임 읽기 & 특징점 추출
    # ==========================================
    curr_frame = me.get_frame_read().frame
    curr_frame = cv2.resize(curr_frame, (w, h)) # 속도를 위해 리사이즈

    # 현재 화면에서 특징점 찾기
    curr_kp, curr_des = orb.detectAndCompute(curr_frame, None)

    # 특징점이 너무 적으면 계산 불가 (그냥 넘어감)
    if prev_des is None or curr_des is None or len(prev_kp) < 10 or len(curr_kp) < 10:
        prev_frame = curr_frame.copy()
        prev_kp, prev_des = curr_kp, curr_des
        continue

    # ==========================================
    # 3. 특징점 매칭 (Matching)
    # ==========================================
    # "아까 봤던 그 점(prev)이 지금 화면(curr)의 어디로 갔니?"
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(prev_des, curr_des)

    # 매칭 결과 정렬 (거리가 가까운 순 = 확실한 매칭 순)
    matches = sorted(matches, key=lambda x: x.distance)

    # 상위 50개 점만 사용 (너무 이상한 매칭은 버리기 위함)
    good_matches = matches[:50]

    # 매칭된 점들의 좌표 추출
    # queryIdx: 이전 프레임의 점 인덱스 / trainIdx: 현재 프레임의 점 인덱스
    pts1 = np.float32([prev_kp[m.queryIdx].pt for m in good_matches])
    pts2 = np.float32([curr_kp[m.trainIdx].pt for m in good_matches])

    # ==========================================
    # 4. 이동 계산 (Essential Matrix & Recover Pose)
    # ==========================================
    # [핵심] 점들의 이동 패턴을 분석해 카메라의 회전(R)과 이동(t)을 계산
    # E: Essential Matrix (본질 행렬)
    E, mask = cv2.findEssentialMat(pts2, pts1, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
    
    if E is not None:
        # E 행렬을 분해하여 회전(R)과 이동(t) 복원
        _, R, t, mask = cv2.recoverPose(E, pts2, pts1, K)

        # --------------------------------------
        # [한계점 보정] 스케일(Scale) 문제
        # --------------------------------------
        # 단안 카메라는 "방향"은 알지만 "얼마나(m) 갔는지"는 모릅니다.
        # 그래서 t(이동벡터)의 크기는 항상 1로 나옵니다.
        # 실제로는 드론의 속도 센서나 IMU 값을 곱해줘야 하지만, 
        # 여기서는 테스트를 위해 '임의의 속도(1.0)'로 움직인다고 가정합니다.
        # 실제 드론이 정지해 있어도 노이즈 때문에 움직인다고 착각할 수 있어 
        # t의 크기가 일정 이상일 때만 움직임으로 간주하는 필터를 넣기도 합니다.
        
        scale = 1.0 # 임의의 스케일 (실제 프로젝트에선 IMU 등으로 대체 필요)
        
        # 좌표 누적 계산: 현재 위치 = 이전 위치 + (회전된 이동방향 * 스케일)
        # P_new = P_old + R * t
        if np.abs(t[2]) > 0.05: # Z축(앞뒤) 움직임이 어느 정도 있을 때만 업데이트 (노이즈 제거)
             curr_t = curr_t + scale * curr_R.dot(t)
             curr_R = R.dot(curr_R)

    # ==========================================
    # 5. 지도 그리기 (Visualization)
    # ==========================================
    # curr_t[0]: X좌표 (좌우), curr_t[2]: Z좌표 (앞뒤)
    # 지도 중심(300, 300)에서 시작하도록 좌표 보정
    draw_x = int(curr_t[0] * 5) + 300 # *5는 궤적을 좀 크게 그리려고 확대
    draw_y = int(curr_t[2] * 5) + 300 # 컴퓨터 비전 좌표계상 Z가 '앞'입니다.

    # 궤적 점 찍기 (초록색)
    cv2.circle(traj_map, (draw_x, draw_y), 1, (0, 255, 0), 2)
    
    # 현재 드론 위치 표시 (빨간색 큰 점)
    map_display = traj_map.copy()
    cv2.circle(map_display, (draw_x, draw_y), 5, (0, 0, 255), -1)
    
    # 정보 텍스트 표시
    text = f"X: {curr_t[0][0]:.2f} Z: {curr_t[2][0]:.2f}"
    cv2.putText(map_display, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # 화면 출력
    cv2.imshow("Drone Camera", curr_frame)
    cv2.imshow("Trajectory Map (Top Down)", map_display)

    # ==========================================
    # 6. 다음 루프 준비
    # ==========================================
    prev_frame = curr_frame.copy()
    prev_kp, prev_des = curr_kp, curr_des

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

me.streamoff()
cv2.destroyAllWindows()