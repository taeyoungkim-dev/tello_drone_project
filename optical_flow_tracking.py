import numpy as np
import cv2
from djitellopy import Tello

# ==========================================
# 1. 초기 설정
# ==========================================
me = Tello()
me.connect()
me.streamon()

# LK 광학 흐름 알고리즘을 위한 파라미터 설정
# winSize: 검색 윈도우 크기 (클수록 빠르지만 정밀도 떨어짐)
# maxLevel: 피라미드 레벨 (이미지를 축소해서 큰 움직임도 잡게 함)
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# 추적할 특징점을 찾기 위한 파라미터 (Shi-Tomasi 코너 검출기)
# ORB보다 추적(Tracking)에는 이 방식이 더 안정적이라 자주 쓰입니다.
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# ==========================================
# 2. 첫 번째 프레임 처리 (기준점 잡기)
# ==========================================
print("첫 번째 프레임을 받아옵니다...")
old_frame = me.get_frame_read().frame
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY) # 흑백 변환 (연산 속도업)

# 최초의 특징점들(p0)을 찾습니다.
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# 궤적을 그리기 위한 빈 도화지 생성
mask = np.zeros_like(old_frame)

print("추적 시작! (드론을 천천히 움직여보세요)")

while True:
    # ==========================================
    # 3. 새로운 프레임 가져오기
    # ==========================================
    frame = me.get_frame_read().frame
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ==========================================
    # 4. 광학 흐름 계산 (핵심!)
    # ==========================================
    # p0(이전 점들)가 frame_gray(새 이미지)에서 어디(p1)로 갔는지 계산
    # st: 상태 (1=찾음, 0=못찾음)
    # err: 오차
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # ==========================================
    # 5. 점 선택 및 그리기
    # ==========================================
    if p1 is not None:
        # 잘 추적된 점들만 골라냅니다 (st==1 인 것들)
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # 궤적 그리기
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel() # 현재 위치
            c, d = old.ravel() # 과거 위치
            
            # 정수로 변환 (픽셀 좌표는 정수여야 함)
            a, b = int(a), int(b)
            c, d = int(c), int(d)

            # 과거 위치에서 현재 위치로 선 그리기 (궤적)
            mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
            # 현재 위치에 점 찍기
            frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)

        # 화면 업데이트
        img = cv2.add(frame, mask)
        cv2.imshow('Optical Flow Tracking', img)

    # ==========================================
    # 6. 다음 루프 준비
    # ==========================================
    # "현재"는 다음 루프에서 "과거"가 됩니다.
    old_gray = frame_gray.copy()
    
    # 점들을 업데이트 (놓친 점은 버리고, 살아남은 점만 p0로 만듦)
    p0 = good_new.reshape(-1, 1, 2)
    
    # 점이 너무 적으면(예: 10개 미만) 다시 찾습니다. (Re-initialization)
    if len(p0) < 10:
        p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
        mask = np.zeros_like(old_frame) # 궤적도 초기화 (너무 지저분해지니까)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

me.streamoff()
cv2.destroyAllWindows()