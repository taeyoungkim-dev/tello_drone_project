import cv2
import os
from djitellopy import Tello
import time

# ==========================================
# ⚙️ 설정 (체커보드 규격)
# ==========================================
# 아까 PDF 만들 때 설정한 값과 일치해야 합니다!
CHECKERBOARD = (9, 6) # 내부 코너 개수 (가로 10칸 -> 내부 9개, 세로 7칸 -> 내부 6개)
SAVE_FOLDER = "calibration_images"

# ==========================================
# 초기화
# ==========================================
if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)
    print(f"📂 '{SAVE_FOLDER}' 폴더 생성 완료")

print("드론 연결 중...")
me = Tello()
me.connect()
print(f"배터리: {me.get_battery()}%")
me.streamon()

count = 0
print("🎥 촬영 시작! 체커보드가 인식되면 무지개 선이 생깁니다.")
print("💾 [S] 키: 사진 저장")
print("❌ [Q] 키: 종료")

while True:
    img = me.get_frame_read().frame
    img = cv2.resize(img, (960, 720)) # Tello 원본 해상도 권장 (화질이 좋아야 인식 잘됨)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 보여주기용 이미지 복사
    display_img = img.copy()

    # 체커보드 찾기
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        # 찾았으면 그리기 (무지개 선)
        cv2.drawChessboardCorners(display_img, CHECKERBOARD, corners, ret)
        
        # 안내 문구 (초록색)
        cv2.putText(display_img, "DETECTED! Press 'S' to Save", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    else:
        cv2.putText(display_img, "Looking for chessboard...", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 저장된 개수 표시
    cv2.putText(display_img, f"Saved: {count}", (50, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    cv2.imshow("Calibration Capture", display_img)

    key = cv2.waitKey(1) & 0xFF
    
    # 's' 누르면 저장 (인식 성공 여부 상관없이 저장 가능하도록 함, 나중에 거르면 됨)
    if key == ord('s'):
        filename = f"{SAVE_FOLDER}/img_{count:03d}.jpg"
        cv2.imwrite(filename, img) # 원본 이미지 저장 (낙서 없는 거)
        print(f"✅ 저장됨: {filename}")
        count += 1
        time.sleep(0.2) # 중복 저장 방지 딜레이
        
    elif key == ord('q'):
        break

me.streamoff()
cv2.destroyAllWindows()