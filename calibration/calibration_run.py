import cv2
import numpy as np
import os
import glob

# ==========================================
# ⚙️ 설정 (매우 중요)
# ==========================================
CHECKERBOARD = (9, 6) # 내부 코너 개수 (가로-1, 세로-1)
SQUARE_SIZE = 0.025   # 정사각형 한 변의 길이 (미터 단위) -> 25mm = 0.025m
IMAGE_FOLDER = "calibration_images"

# ==========================================
# 캘리브레이션 준비
# ==========================================
# 3D 점들의 좌표 정의 (0,0,0), (1,0,0), (2,0,0) ...., (8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE # 실제 크기 반영

# 3D 점(Real World)과 2D 점(Image)을 담을 리스트
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob(f'{IMAGE_FOLDER}/*.jpg')

if len(images) == 0:
    print("❌ 오류: 이미지가 없습니다. 촬영부터 하세요!")
    exit()

print(f"🔍 {len(images)}장의 이미지를 분석합니다...")
valid_images = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 체커보드 코너 찾기
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        valid_images += 1
        # 정밀도를 높이기 위해 코너 위치 미세 조정 (Subpixel)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        
        objpoints.append(objp)
        imgpoints.append(corners2)
        print(f" - {fname}: OK")
    else:
        print(f" - {fname}: 실패 (코너 못 찾음)")

print(f"\n📊 분석 완료: 총 {valid_images}장 사용 가능")

if valid_images < 10:
    print("⚠️ 경고: 유효한 이미지가 너무 적습니다. 정확도가 떨어질 수 있습니다.")

# ==========================================
# 캘리브레이션 실행 (수학 계산)
# ==========================================
print("🧮 캘리브레이션 계산 중... (잠시만 기다리세요)")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# ==========================================
# 결과 출력 (이걸 복사해서 쓰세요!)
# ==========================================
print("\n" + "="*50)
print("✅ Tello Camera Calibration Result")
print("="*50)
print(f"Reprojection Error: {ret:.4f} (0에 가까울수록 좋음, 1.0 미만이면 성공)")
print("-" * 30)
print("👇 아래 코드를 복사해서 프로젝트에 붙여넣으세요 👇")
print("-" * 30)
print("import numpy as np")
print("\n# Tello Camera Matrix (Intrinsic)")
print("camera_matrix = np.array([")
print(f"    [{mtx[0][0]:.8f}, {mtx[0][1]:.8f}, {mtx[0][2]:.8f}],")
print(f"    [{mtx[1][0]:.8f}, {mtx[1][1]:.8f}, {mtx[1][2]:.8f}],")
print(f"    [{mtx[2][0]:.8f}, {mtx[2][1]:.8f}, {mtx[2][2]:.8f}]")
print("], dtype=np.float32)")

print("\n# Distortion Coefficients")
print(f"dist_coeffs = np.array([{', '.join([f'{x:.8f}' for x in dist[0]])}], dtype=np.float32)")
print("="*50)

# 결과 저장 (npz 파일)
np.savez("tello_calibration_data.npz", mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
print("💾 데이터가 'tello_calibration_data.npz' 파일로도 저장되었습니다.")