import cv2
from djitellopy import Tello
from ultralytics import YOLO
import time

# --- 설정 파트 ---
w, h = 480, 360
# PID 게인 [Yaw, Up/Down, Forward/Back(Distance)]
# 거리 제어용 Kp(0.002)는 아래 함수 내에서 하드코딩하거나 별도 변수로 관리
pid_yaw = [0.4, 0.4, 0]
pid_ud = [0.4, 0.4, 0]

# P 제어 목표값 (Setpoint)
target_area = 45000  # 드론이 유지하려는 사람 크기 (거리)

print("YOLO 모델 로딩 중...")
model = YOLO('yolov8n.pt')
print("모델 로드 완료!")

me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon()
me.takeoff()
# 초기 상승 속도를 40 -> 25로 낮추고, 대기 시간도 줄여서 안전 확보
me.send_rc_control(0, 0, 25, 0)
time.sleep(1.5)

def findPerson(img):
    # conf=0.65 유지
    results = model(img, stream=True, classes=0, verbose=False, conf=0.65)
    
    personListC = []
    personListArea = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            w_box = x2 - x1
            h_box = y2 - y1
            cx = x1 + w_box // 2
            cy = y1 + h_box // 2
            area = w_box * h_box
            
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
            
            personListC.append([cx, cy])
            personListArea.append(area)
            
    if len(personListArea) != 0:
        i = personListArea.index(max(personListArea))
        return img, [personListC[i], personListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackPerson(info, w, h, pid_yaw, pid_ud):
    area = info[1]
    x, y = info[0]
    
    # [안전 장치] 사람을 못 찾았으면(x=0) 모든 계산 중단 및 정지
    if x == 0:
        me.send_rc_control(0, 0, 0, 0)
        return 0 # Error 0 리턴
    
    # 1. Yaw (회전) P 제어
    error_x = x - w // 2
    speed_yaw = pid_yaw[0] * error_x
    speed_yaw = int(max(-100, min(speed_yaw, 100))) 

    # 2. Up/Down (고도) P 제어
    # 사람이 없으면(x=0) 위에서 이미 리턴했으므로, 여기선 y=0일 걱정 없음
    error_y = (h // 2) - y
    speed_ud = pid_ud[0] * error_y
    speed_ud = int(max(-100, min(speed_ud, 100)))

    # 3. Distance (거리) P 제어 - 핵심 수정 부분!
    # Error = 목표 면적 - 현재 면적
    # 예: 목표(45000) - 현재(20000, 멂) = +25000 -> 전진 필요 (속도 양수)
    # 예: 목표(45000) - 현재(60000, 가깝) = -15000 -> 후진 필요 (속도 음수)
    error_dist = target_area - area
    
    # Kp 게인: 0.002 (면적 단위가 크므로 아주 작은 값 사용)
    # 25000 * 0.002 = 50 (속도)
    kp_dist = 0.002 
    speed_fb = kp_dist * error_dist
    speed_fb = int(max(-100, min(speed_fb, 100)))
    
    # [Deadzone 설정] 미세한 떨림 방지를 위해 속도가 작으면 0으로 무시
    if abs(speed_fb) < 5: speed_fb = 0
    if abs(speed_ud) < 5: speed_ud = 0
    if abs(speed_yaw) < 5: speed_yaw = 0

    # 최종 명령 전송
    me.send_rc_control(0, speed_fb, speed_ud, speed_yaw)
    
    # 디버깅을 위해 거리 오차 리턴
    return error_dist

# 메인 루프
while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))
    
    img, info = findPerson(img)
    
    dist_err = trackPerson(info, w, h, pid_yaw, pid_ud)
    
    # 화면 정보 표시
    cv2.putText(img, f"Area: {info[1]}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
    cv2.putText(img, f"DistErr: {dist_err}", (10, 60), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)

    cv2.imshow("Final P-Control Tracking", img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

cv2.destroyAllWindows()
