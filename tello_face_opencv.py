import cv2
from djitellopy import Tello
import time

# --- 설정 파트 ---
w, h = 360, 240  # 연산 속도를 위해 이미지 크기 축소
pid = [0.4, 0.4, 0]  # P, I, D 게인 (여기선 P만 사용: 0.4)
pError = 0  # 이전 오차 (D제어용, 현재는 사용 안 함)
startCounter = 0  # 이륙 전 대기 카운터

# 드론 연결 및 초기화
me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon() # 비디오 스트림 시작
me.takeoff()  # !!! 주의: 코드 실행 시 바로 이륙합니다 !!!
me.send_rc_control(0, 0, 25, 0) # 처음엔 눈높이까지 살짝 상승
time.sleep(2.2) # 상승 대기

# OpenCV 얼굴 인식 모델 로드 (경로 자동 설정)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def findFace(img):
    faceList = []
    myFaceListC = []
    myFaceListArea = []
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    
    for (x, y, w_box, h_box) in faces:
        # 얼굴에 사각형 그리기
        cv2.rectangle(img, (x, y), (x + w_box, y + h_box), (0, 0, 255), 2)
        
        # 얼굴 중심점(cx, cy) 계산
        cx = x + w_box // 2
        cy = y + h_box // 2
        area = w_box * h_box
        
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
        faceList.append([x, y, w_box, h_box])
    
    if len(myFaceListArea) != 0:
        # 가장 가까운(영역이 가장 큰) 얼굴 하나만 추적
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackFace(info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    fb = 0 # Forward/Backward Speed
    
    # 1. Yaw 제어 (회전)
    # 화면 중심(w//2)과 얼굴 중심(x)의 오차 계산
    error = x - w // 2
    # P 제어: 오차 * 게인 -> 속도 결정 (값 클램핑 -100~100)
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(max(-100, min(speed, 100))) 
    
    # 얼굴이 감지되지 않았으면(x=0) 회전 멈춤
    if x == 0:
        speed = 0
        error = 0
    
    # 2. Pitch 제어 (거리 유지)
    # 얼굴 영역(area)이 일정 범위(6000~10000) 내에 들어오도록 제어
    # 너무 가까우면(>10000) 후진, 멀면(<6000) 전진
    if area > 6000 and area < 10000:
        fb = 0
    elif area > 10000:
        fb = -20 # 후진
    elif area < 6000 and area != 0:
        fb = 20  # 전진
    
    # 얼굴 없으면 제자리 정지
    if x == 0:
        fb = 0
        error = 0
        
    # 드론에 명령 전송 (좌우이동, 전후이동, 상하이동, 회전)
    me.send_rc_control(0, fb, 0, speed)
    return error

# 메인 루프
while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w, h))
    
    # 얼굴 찾기
    img, info = findFace(img)
    
    # 트래킹 제어
    pError = trackFace(info, w, pid, pError)
    
    cv2.imshow("Tello Face Tracking", img)
    
    # 'q' 키를 누르면 착륙 후 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        break

cv2.destroyAllWindows()