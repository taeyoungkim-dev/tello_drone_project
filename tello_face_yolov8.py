import cv2
from djitellopy import Tello
from ultralytics import YOLO
import time
import math

# --- 설정 파트 ---
w, h = 480, 360  # 추론 속도와 화질의 타협점
pid = [0.4, 0.4, 0]  # P, I, D 게인
pError = 0  # 이전 오차

# YOLO 모델 로드 (최초 실행 시 자동 다운로드)
# 'n'은 nano 버전으로 가장 빠릅니다.
print("YOLO 모델 로딩 중...")
model = YOLO('yolov8n.pt') 
print("모델 로드 완료!")

# 드론 연결
me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon()
me.takeoff()
me.send_rc_control(0, 0, 20, 0) # 눈높이 상승
time.sleep(2.0)

def findPerson(img):
    # YOLO 추론 (classes=0은 'Person' 클래스만 검출)
    results = model(img, stream=True, classes=0, verbose=False, conf=0.7)
    
    personListC = []
    personListArea = []
    
    # 결과 파싱
    for r in results:
        boxes = r.boxes
        for box in boxes:
            # 바운딩 박스 좌표
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 중심점 및 면적 계산
            w_box = x2 - x1
            h_box = y2 - y1
            cx = x1 + w_box // 2
            cy = y1 + h_box // 2
            area = w_box * h_box
            
            # 시각화
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
            
            personListC.append([cx, cy])
            personListArea.append(area)
            
    if len(personListArea) != 0:
        # 가장 큰(가까운) 사람 선택
        i = personListArea.index(max(personListArea))
        return img, [personListC[i], personListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackPerson(info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    fb = 0
    
    # 1. Yaw (회전) 제어
    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(max(-100, min(speed, 100))) 
    
    # 2. Pitch (거리) 제어
    # YOLO는 전신을 잡으므로 면적 기준을 조금 더 크게 잡습니다
    # area 값은 테스트하면서 조절 필요 (현재: 15000 ~ 25000 기준)
    if x == 0: # 사람을 놓치면
        speed = 0
        error = 0
        fb = 0
    else:
        # 사람이 너무 가까우면 (면적이 40,000 초과) -> 후진
        if area > 40000:
            fb = -20 
        # 사람이 적당한 거리에 있으면 (30,000 ~ 40,000) -> 정지
        elif area > 30000 and area < 40000:
            fb = 0
        # 사람이 멀면 (면적이 30,000 미만) -> 전진
        elif area < 30000:
            fb = 20
        
    me.send_rc_control(0, fb, 0, speed)
    return error

# 메인 루프
while True:
    img = me.get_frame_read().frame
    # YOLO 성능 최적화를 위해 이미지 리사이즈
    img = cv2.resize(img, (w, h))
    
    img, info = findPerson(img)
    pError = trackPerson(info, w, pid, pError)
    
    cv2.imshow("YOLO Tello Tracking", img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

cv2.destroyAllWindows()