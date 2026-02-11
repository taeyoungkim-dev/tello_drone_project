import cv2
from djitellopy import Tello
from ultralytics import YOLO
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
# [비디오 저장 기능 추가] 비디오 저장 전용 스레드
# ==========================================
class VideoWriter(threading.Thread):
    """
    백그라운드에서 비디오를 저장하는 스레드
    - 메인 루프 성능에 영향 없이 비디오 저장
    - 큐 기반 비동기 처리로 디스크 I/O 블로킹 방지
    """
    def __init__(self, filename, width, height, fps=20.0, codec='XVID'):
        super().__init__()
        self.daemon = True
        self.frame_queue = queue.Queue(maxsize=100)  # 최대 100프레임 버퍼
        self.filename = filename
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec
        self.running = True
        self.frames_written = 0
        
    def run(self):
        # 비디오 라이터 초기화
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        writer = cv2.VideoWriter(self.filename, fourcc, self.fps, (self.width, self.height))
        
        if not writer.isOpened():
            print(f"⚠️ 경고: 비디오 파일 생성 실패 ({self.filename})")
            return
        
        print(f"🎥 비디오 저장 시작: {self.filename} ({self.codec} 코덱, {self.fps} FPS)")
        
        while self.running or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.1)
                writer.write(frame)
                self.frames_written += 1
            except queue.Empty:
                continue
        
        # 비디오 라이터 종료
        writer.release()
        print(f"✅ 비디오 저장 완료: {self.frames_written}개 프레임")
    
    def write(self, frame):
        """
        프레임을 큐에 추가 (매우 빠름, 1μs 미만)
        큐가 꽉 찼으면 프레임 드롭 (성능 보호)
        """
        if not self.frame_queue.full():
            self.frame_queue.put(frame)
        else:
            # 큐가 꽉 찼으면 프레임 드롭 (경고 없이 무시)
            pass
    
    def stop(self):
        """비디오 저장 스레드 종료"""
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
        
        # ★ YOLO는 느리기 때문에 큐 관리가 더 중요합니다.
        #    추론이 끝났을 때 무조건 '방금 찍은' 사진이 있어야 합니다.
        self.frame_queue = queue.Queue(maxsize=1) 

    def run(self):
        stream_reader = self.tello.get_frame_read()
        while self.running:
            frame = stream_reader.frame
            if frame is None:
                continue

            # 전처리(Resize)를 스레드에서 수행
            frame = cv2.resize(frame, (self.width, self.height))

            # ==========================================
            # [성능 측정 추가] 프레임에 타임스탬프 추가
            # ==========================================
            frame_timestamp = time.time()
            
            # 큐 최신화 (오래된 프레임 Drop)
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
            
            # [성능 측정 추가] 프레임과 타임스탬프를 함께 큐에 넣기
            self.frame_queue.put((frame, frame_timestamp))
            # ==========================================
            time.sleep(0.01)

    def stop(self):
        self.running = False

# ==========================================
# 1. 설정 및 초기화
# ==========================================
w, h = 480, 360
# PID 게인 [Yaw, Up/Down, Forward/Back]
pid_yaw = [0.4, 0.4, 0]
pid_ud = [0.4, 0.4, 0]
target_area = 45000 

# ==========================================
# [안전성 개선] 이륙 전 모델 로드 및 검증
# ==========================================
# 드론이 이륙한 후 모델 로드 실패 시 위험할 수 있으므로
# 반드시 이륙 전에 모델을 로드하고 검증합니다.
print("YOLO 모델 로딩 중... (시간이 좀 걸립니다)")
try:
    model = YOLO('yolov8n.pt')
    
    # 간단한 더미 추론으로 모델 검증 (처음 추론은 초기화 시간 포함)
    import numpy as np
    dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
    _ = model(dummy_img, verbose=False)
    
    print("✅ YOLO 모델 로드 및 초기화 완료!")
    
except Exception as e:
    print(f"❌ YOLO 모델 로드 실패: {e}")
    print("\n다음을 확인하세요:")
    print("  1. ultralytics가 설치되었는지 (pip install ultralytics)")
    print("  2. yolov8n.pt 파일이 존재하거나 다운로드 가능한지")
    print("  3. 인터넷 연결 상태 (모델 자동 다운로드용)")
    print("\n안전을 위해 프로그램을 종료합니다.")
    exit(1)
# ==========================================

print("\n드론 연결 중...")
me = Tello()
me.connect()
print(f"배터리 잔량: {me.get_battery()}%")

me.streamon()

# ★ 스레드 시작 (이륙 전에 영상 수신 시작)
receiver = FrameReceiver(me, w, h)
receiver.start()

# 영상이 들어올 때까지 대기
while receiver.frame_queue.empty():
    time.sleep(0.1)
    print("영상 수신 대기 중...")

print("\n이륙 준비 완료! 3초 뒤 이륙합니다.")
time.sleep(3)

me.takeoff()
me.send_rc_control(0, 0, 25, 0)
time.sleep(1.5)

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
log_filename = LOG_DIR / f"flight_log_yolov8_optimized_{now}.csv"
logger = LogWriter(str(log_filename), flush_interval_sec=1.0)
logger.start()
print(f"📝 로그 파일 생성: {log_filename}")

# ==========================================
# [비디오 저장 기능 추가] 비디오 파일 생성 및 스레드 시작
# ==========================================
# videos 폴더 생성 (없으면 자동 생성)
VIDEO_DIR = Path("videos")
VIDEO_DIR.mkdir(exist_ok=True)

video_filename = VIDEO_DIR / f"yolov8n_{now}.avi"
video_writer = VideoWriter(str(video_filename), w, h, fps=20.0, codec='XVID')
video_writer.start()
print(f"🎥 비디오 파일 생성: {video_filename}")
# ==========================================

frame_count = 0  # 프레임 카운터

# ==========================================
# 2. 함수 정의
# ==========================================
def findPerson(img):
    # stream=True 옵션 사용 (메모리 효율화)
    results = model(img, stream=True, classes=0, verbose=False, conf=0.65)
    
    personListC = []
    personListArea = []
    
    # YOLO 결과 파싱
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
    
    if x == 0:
        me.send_rc_control(0, 0, 0, 0)
        return 0 
    
    # 1. Yaw 제어
    error_x = x - w // 2
    speed_yaw = pid_yaw[0] * error_x
    speed_yaw = int(max(-100, min(speed_yaw, 100))) 

    # 2. Up/Down 제어
    error_y = (h // 2) - y
    speed_ud = pid_ud[0] * error_y
    speed_ud = int(max(-100, min(speed_ud, 100)))

    # 3. Distance 제어
    error_dist = target_area - area
    kp_dist = 0.002 
    speed_fb = kp_dist * error_dist
    speed_fb = int(max(-100, min(speed_fb, 100)))
    
    # Deadzone
    if abs(speed_fb) < 5: speed_fb = 0
    if abs(speed_ud) < 5: speed_ud = 0
    if abs(speed_yaw) < 5: speed_yaw = 0

    me.send_rc_control(0, speed_fb, speed_ud, speed_yaw)
    return error_dist

# ==========================================
# 3. 메인 루프 (Consumer)
# ==========================================
try:
    while True:
        # ★ [핵심] YOLO가 아무리 느려도, 여기서 get() 하는 순간
        #    스레드가 넣어둔 0.01초 전 '최신 영상'을 가져옴.
        #    즉, '추론 시간'은 걸리지만 '데이터 지연'은 사라짐.
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
        
        # YOLO 추론 & 제어
        img, info = findPerson(img)
        dist_err = trackPerson(info, w, h, pid_yaw, pid_ud)
        
        cv2.putText(img, f"Area: {info[1]}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
        cv2.putText(img, f"DistErr: {dist_err}", (10, 60), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
        
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
        
        # ==========================================
        # [비디오 저장 기능 추가] 처리된 프레임 저장
        # ==========================================
        video_writer.write(img)
        # ==========================================

        cv2.imshow("YOLOv8 Optimization Tracking", img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            break

except KeyboardInterrupt:
    me.land()

finally:
    receiver.stop()
    receiver.join()
    
    # ==========================================
    # [로깅 기능 추가] 로그 스레드 종료 및 파일 저장
    # ==========================================
    logger.stop()
    logger.join()
    print(f"✅ 로그 저장 완료: {log_filename} (총 {frame_count}개 프레임)")
    # ==========================================
    
    # ==========================================
    # [비디오 저장 기능 추가] 비디오 저장 스레드 종료
    # ==========================================
    video_writer.stop()
    video_writer.join()
    print(f"✅ 비디오 저장 완료: {video_filename}")
    # ==========================================
    
    me.streamoff()
    cv2.destroyAllWindows()
    print("종료되었습니다.")