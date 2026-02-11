import cv2
from djitellopy import Tello
import time
import threading
import queue
import numpy as np
# ==========================================
# [로깅 기능 추가] CSV 로그 저장용 임포트
# ==========================================
import csv
import datetime
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

# ==========================================
# [CS 핵심] 영상 수신 전용 쓰레드 (Producer)
# ==========================================
class FrameReceiver(threading.Thread):
    def __init__(self, tello, width, height):
        threading.Thread.__init__(self)
        self.tello = tello
        self.width = width
        self.height = height
        self.daemon = True 
        self.running = True
        
        # ★ 가장 중요한 부분: 큐 크기를 1로 제한
        self.frame_queue = queue.Queue(maxsize=1) 

    def run(self):
        stream_reader = self.tello.get_frame_read()
        while self.running:
            frame = stream_reader.frame
            if frame is None:
                continue

            # 전처리(Resize)를 여기서 수행해 메인 스레드 부담 경감
            frame = cv2.resize(frame, (self.width, self.height))

            # ==========================================
            # [성능 측정 추가] 프레임에 타임스탬프 추가
            # ==========================================
            frame_timestamp = time.time()
            
            # 큐가 꽉 찼으면(1개 있으면) 옛날 거 버리고 새거 넣기
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            # [성능 측정 추가] 프레임과 타임스탬프를 함께 큐에 넣기
            self.frame_queue.put((frame, frame_timestamp))
            # ==========================================
            time.sleep(0.01) # CPU 점유율 조절

    def stop(self):
        self.running = False

# ==========================================
# 1. 설정 및 초기화
# ==========================================
w, h = 360, 240
pid = [0.4, 0.4, 0]
pError = 0

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

print("\n드론 연결 및 초기화 중...")
me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon()

# ★ 스레드 시작 (이륙 전에 미리 영상 받아오기 시작)
receiver = FrameReceiver(me, w, h)
receiver.start()

# 영상이 들어올 때까지 잠시 대기 (안전장치)
while receiver.frame_queue.empty():
    time.sleep(0.1)
    print("영상 수신 대기 중...")

print("\n이륙 준비 완료! 3초 뒤 이륙합니다.")
time.sleep(3) 

me.takeoff()
me.send_rc_control(0, 0, 25, 0) # 눈높이 상승
time.sleep(2.2)

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
log_filename = LOG_DIR / f"flight_log_opencv_optimized_{now}.csv"
logger = LogWriter(str(log_filename), flush_interval_sec=1.0)
logger.start()
print(f"📝 로그 파일 생성: {log_filename}")

frame_count = 0  # 프레임 카운터

# ==========================================
# 2. 함수 정의 (기존 로직 유지)
# ==========================================
def findFace(img):
    faceList = []
    myFaceListC = []
    myFaceListArea = []
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    
    for (x, y, w_box, h_box) in faces:
        cv2.rectangle(img, (x, y), (x + w_box, y + h_box), (0, 0, 255), 2)
        cx = x + w_box // 2
        cy = y + h_box // 2
        area = w_box * h_box
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
        
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)
        faceList.append([x, y, w_box, h_box])
    
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]

def trackFace(info, w, pid, pError):
    area = info[1]
    x, y = info[0]
    fb = 0
    
    error = x - w // 2
    speed = pid[0] * error + pid[1] * (error - pError)
    speed = int(max(-100, min(speed, 100))) 
    
    if x == 0:
        speed = 0
        error = 0
    
    if area > 6000 and area < 10000:
        fb = 0
    elif area > 10000:
        fb = -20
    elif area < 6000 and area != 0:
        fb = 20
    
    if x == 0:
        fb = 0
        error = 0
        
    me.send_rc_control(0, fb, 0, speed)
    return error

# ==========================================
# 3. 메인 루프 (Consumer)
# ==========================================
try:
    while True:
        # ★ [핵심 변경] 드론한테 직접 달라고 안 하고, 큐에서 '가장 최신'꺼 꺼내옴
        # 메인 루프가 얼굴 인식하느라 0.1초 늦어져도, 
        # receiver가 이미 0.01초 전 사진을 큐에 넣어뒀음.
        try:
            # ==========================================
            # [성능 측정 추가] 프레임과 타임스탬프를 함께 받기
            # ==========================================
            img, frame_timestamp = receiver.frame_queue.get(timeout=1.0)
        except queue.Empty:
            continue

        # ==========================================
        # [성능 측정 개선] 처리 시작 시간 기록
        # ==========================================
        processing_start_time = time.time()
        
        # 얼굴 찾기 & 제어 (로직 동일)
        img, info = findFace(img)
        pError = trackFace(info, w, pid, pError)
        
        # ==========================================
        # [성능 측정 개선] FPS 및 Frame Latency 계산
        # ==========================================
        cTime = time.time()
        
        # 순수 처리 시간 기반 FPS (큐 대기 시간 제외)
        processing_time = cTime - processing_start_time
        fps = 1 / processing_time if processing_time > 0 else 0
        
        # Frame Latency: 프레임 수신 시점부터 처리 시작까지의 지연 (ms 단위)
        # 이는 프레임이 큐에서 대기한 시간을 나타냄
        frame_latency = (processing_start_time - frame_timestamp) * 1000
        
        pTime = cTime
        
        # 화면에 성능 지표 표시
        cv2.putText(img, f"Loop FPS: {int(fps)}", (10, h - 70), 
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
        cv2.putText(img, f"Frame Age: {int(frame_latency)}ms", (10, h - 40), 
                    cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
        # ==========================================
        
        # ==========================================
        # [로깅 기능 추가] 성능 데이터 로그 저장
        # ==========================================
        frame_count += 1
        logger.log([frame_count, cTime, frame_latency, fps])
        # ==========================================
        
        cv2.imshow("Tello Face Tracking (Optimized)", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            break

except KeyboardInterrupt:
    me.land()

finally:
    receiver.stop() # 스레드 종료
    receiver.join()
    
    # ==========================================
    # [로깅 기능 추가] 로그 스레드 종료 및 파일 저장
    # ==========================================
    logger.stop()
    logger.join()
    print(f"✅ 로그 저장 완료: {log_filename} (총 {frame_count}개 프레임)")
    # ==========================================
    
    me.streamoff()
    cv2.destroyAllWindows()
    print("종료되었습니다.")