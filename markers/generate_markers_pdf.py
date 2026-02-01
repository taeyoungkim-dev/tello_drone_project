import cv2
import os
from fpdf import FPDF

# ==========================================
# ⚙️ 사용자 설정 (여기만 고치세요!)
# ==========================================
MARKER_REAL_SIZE_MM = 100  # 🌟 원하는 실제 인쇄 크기 (mm 단위) -> 100mm = 10cm
TOTAL_MARKERS = 50         # 0번부터 49번까지
DICT_TYPE = cv2.aruco.DICT_4X4_50 # 마커 딕셔너리
OUTPUT_FILENAME = "markers_exact_size.pdf"

# A4 용지 설정
A4_WIDTH_MM = 210
A4_HEIGHT_MM = 297
MARGIN_MM = 10 

# ==========================================
# 로직 시작
# ==========================================
class PDF(FPDF):
    def footer(self):
        self.set_y(-15)
        self.set_font('Arial', 'I', 8)
        self.cell(0, 10, f'Page {self.page_no()}', 0, 0, 'C')

def create_marker_pdf():
    # 1. PDF 객체 생성 (단위: mm, 크기: A4)
    pdf = PDF(orientation='P', unit='mm', format='A4')
    pdf.set_auto_page_break(auto=True, margin=MARGIN_MM)
    
    # 딕셔너리 로드
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
    
    # 한 줄에 몇 개나 들어갈지 계산
    cols = int((A4_WIDTH_MM - (MARGIN_MM * 2)) // MARKER_REAL_SIZE_MM)
    rows = int((A4_HEIGHT_MM - (MARGIN_MM * 2)) // (MARKER_REAL_SIZE_MM + 10)) # 10mm는 텍스트 여유분
    
    if cols < 1 or rows < 1:
        print("❌ 오류: 마커 크기가 너무 커서 A4 용지에 안 들어갑니다.")
        return

    print(f"📄 PDF 생성을 시작합니다... (크기: {MARKER_REAL_SIZE_MM}mm)")
    print(f"📐 배치: 가로 {cols}개 x 세로 {rows}개")

    pdf.add_page()
    pdf.set_font("Arial", size=12)

    temp_img_name = "temp_marker.png"
    
    # 그리드 인덱스
    curr_col = 0
    curr_row = 0

    for marker_id in range(TOTAL_MARKERS):
        # -------------------------------------
        # 1. OpenCV로 마커 이미지 생성
        # -------------------------------------
        # 해상도는 인쇄 품질을 위해 크게 잡음 (픽셀 크기는 PDF mm 크기와 상관없음)
        img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 1000)
        # 테두리 추가 (인식률 향상용 흰색 여백) -> PDF에 그릴 때 포함됨
        img = cv2.copyMakeBorder(img, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
        
        # 임시 파일 저장 (fpdf가 파일 경로를 요구함)
        cv2.imwrite(temp_img_name, img)

        # -------------------------------------
        # 2. PDF에 이미지 삽입 (핵심!)
        # -------------------------------------
        # 좌표 계산
        x_pos = MARGIN_MM + (curr_col * (MARKER_REAL_SIZE_MM + 5))
        y_pos = MARGIN_MM + (curr_row * (MARKER_REAL_SIZE_MM + 15))

        # 이미지 넣기 (w와 h에 우리가 원하는 mm 단위를 넣으면 됨)
        pdf.image(temp_img_name, x=x_pos, y=y_pos, w=MARKER_REAL_SIZE_MM, h=MARKER_REAL_SIZE_MM)
        
        # ID 텍스트 넣기 (마커 아래에 배치)
        pdf.text(x_pos, y_pos + MARKER_REAL_SIZE_MM + 3, f"ID: {marker_id} ({MARKER_REAL_SIZE_MM}mm)")
        
        # 다음 위치 계산
        curr_col += 1
        if curr_col >= cols:
            curr_col = 0
            curr_row += 1
            
            # 페이지가 꽉 찼으면 다음 페이지로
            if curr_row >= rows and marker_id < TOTAL_MARKERS - 1:
                pdf.add_page()
                curr_row = 0

    # 임시 파일 삭제
    if os.path.exists(temp_img_name):
        os.remove(temp_img_name)
        
    # PDF 저장
    pdf.output(OUTPUT_FILENAME)
    print(f"✅ 완료! '{OUTPUT_FILENAME}' 파일이 생성되었습니다.")

if __name__ == "__main__":
    create_marker_pdf()