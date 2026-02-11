import cv2
from djitellopy import Tello
import time
# ==========================================
# [로깅 기능 추가] CSV 로그 저장용 임포트
# ==========================================
import csv
import datetime
import threading
import queue
from pathlib import Path

# ==========================================
# [로깅 기능 추가] CSV 로그 저장 전용 스레드
# ==========================================
class LogWriter(threading.Thread):
    """
    백그라운드에서 로그를 CSV 파일로 저장하는 스레드
    - 메인 루프 성능에 영향 없이 로그 기록
    - 시간 기반 flush로 디스크 I/O 최소화 (1초마다)
    """
    def __init__(self, filename, flush_interval_sec=1.0):
        super().__init__()
        self.daemon = True
        self.log_queue = queue.Queue()
        self.filename = filename
        self.running = True
        self.flush_interval_sec = flush_interval_sec
        
    def run(self):
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # CSV 헤더 작성
            writer.writerow(["FrameCount", "Timestamp", "Latency_ms", "FPS"])
            
            last_flush = time.time()
            
            while self.running or not self.log_queue.empty():
                try:
                    data = self.log_queue.get(timeout=0.1)
                    writer.writerow(data)
                    
                    # 지정된 시간마다만 flush (디스크 I/O 최소화)
                    if time.time() - last_flush >= self.flush_interval_sec:
                        f.flush()
                        last_flush = time.time()
                        
                except queue.Empty:
                    # 큐가 비어도 주기적으로 flush
                    if time.time() - last_flush >= self.flush_interval_sec:
                        f.flush()
                        last_flush = time.time()
                    continue
            
            # 종료 시 남은 데이터 flush
            f.flush()
    
    def log(self, data):
        """로그 데이터를 큐에 추가 (매우 빠름, 1μs 미만)"""
        self.log_queue.put(data)
    
    def stop(self):
        """로그 스레드 종료"""
        self.running = False

# --- 설정 파트 ---
w, h = 360, 240  # 연산 속도를 위해 이미지 크기 축소
pid = [0.4, 0.4, 0]  # P, I, D 게인 (여기선 P만 사용: 0.4)
pError = 0  # 이전 오차 (D제어용, 현재는 사용 안 함)
startCounter = 0  # 이륙 전 대기 카운터

# ==========================================
# [안전성 개선] 이륙 전 모델 로드 및 검증
# ==========================================
# 드론이 이륙한 후 모델 로드 실패 시 위험할 수 있으므로
# 반드시 이륙 전에 모델을 로드하고 검증합니다.
print("얼굴 인식 모델 로딩 중...")
try:
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # 모델이 제대로 로드되었는지 검증
    if face_cascade.empty():
        raise Exception("CascadeClassifier가 비어있습니다. XML 파일을 확인하세요.")
    
    print("✅ 얼굴 인식 모델 로드 완료!")
    
except Exception as e:
    print(f"❌ 모델 로드 실패: {e}")
    print("\n다음을 확인하세요:")
    print("  1. OpenCV가 올바르게 설치되었는지 (pip install opencv-python)")
    print("  2. haarcascade XML 파일이 존재하는지")
    print("\n안전을 위해 프로그램을 종료합니다.")
    exit(1)
# ==========================================

# 드론 연결 및 초기화
print("\n드론 연결 중...")
me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon() # 비디오 스트림 시작

print("\n⚠️  주의: 3초 뒤 자동으로 이륙합니다!")
time.sleep(3)

me.takeoff()  # !!! 주의: 코드 실행 시 바로 이륙합니다 !!!
me.send_rc_control(0, 0, 25, 0) # 처음엔 눈높이까지 살짝 상승
time.sleep(2.2) # 상승 대기

# ==========================================
# [성능 측정 추가] FPS 계산용 변수
# ==========================================
pTime = 0  # 이전 프레임 시간

# ==========================================
# [로깅 기능 추가] 로그 파일 생성 및 스레드 시작
# ==========================================
# logs 폴더 생성 (없으면 자동 생성)
LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
log_filename = LOG_DIR / f"flight_log_opencv_{now}.csv"
logger = LogWriter(str(log_filename), flush_interval_sec=1.0)
logger.start()
print(f"📝 로그 파일 생성: {log_filename}")

frame_count = 0  # 프레임 카운터

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
try:
    while True:
        # ==========================================
        # [성능 측정 개선] 순수 처리 시간 및 Frame Latency 측정
        # ==========================================
        # 루프 시작 시간 기록 (프레임 대기 시간 포함)
        loop_start_time = time.time()
        
        img = me.get_frame_read().frame
        
        # 프레임 수신 직후 타임스탬프 기록 (Frame Latency 계산용)
        frame_timestamp = time.time()
        
        img = cv2.resize(img, (w, h))
        
        # 얼굴 찾기
        img, info = findFace(img)
        
        # 트래킹 제어
        pError = trackFace(info, w, pid, pError)
        
        # ==========================================
        # [성능 측정 개선] FPS 및 Frame Latency 계산
        # ==========================================
        cTime = time.time()
        
        # 순수 처리 시간 기반 FPS (프레임 대기 시간 제외)
        processing_time = cTime - frame_timestamp
        fps = 1 / processing_time if processing_time > 0 else 0
        
        # Frame Latency: 루프 시작부터 처리 시작까지의 지연 (ms 단위)
        # 이는 WiFi 수신 대기 시간을 나타냄
        frame_latency = (frame_timestamp - loop_start_time) * 1000
        
        pTime = cTime
        
        # 화면에 성능 지표 표시
        cv2.putText(img, f"Loop FPS: {int(fps)}", (10, h - 70), 
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(img, f"WiFi Delay: {int(frame_latency)}ms", (10, h - 40), 
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        # ==========================================
        
        # ==========================================
        # [로깅 기능 추가] 성능 데이터 로그 저장
        # ==========================================
        frame_count += 1
        # Frame Latency는 WiFi 수신 대기 시간 (프레임 가져오는 시간)
        logger.log([frame_count, cTime, frame_latency, fps])
        # ==========================================
        
        cv2.imshow("Tello Face Tracking", img)
        
        # 'q' 키를 누르면 착륙 후 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            break

except KeyboardInterrupt:
    print("\n키보드 인터럽트 감지")
    me.land()

finally:
    # ==========================================
    # [로깅 기능 추가] 로그 스레드 종료 및 파일 저장
    # ==========================================
    logger.stop()
    logger.join()
    print(f"✅ 로그 저장 완료: {log_filename} (총 {frame_count}개 프레임)")
    # ==========================================
    
    cv2.destroyAllWindows()
    print("프로그램 종료")