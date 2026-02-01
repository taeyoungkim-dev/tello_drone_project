import os
from fpdf import FPDF

# ==========================================
# ⚙️ 설정 (Calibration Board Config)
# ==========================================
SQUARE_SIZE_MM = 25  # 정사각형 한 변의 길이 (25mm = 2.5cm 추천)
ROWS = 7             # 세로 사각형 개수
COLS = 10            # 가로 사각형 개수
OUTPUT_FILENAME = "calibration_chessboard.pdf"

# A4 용지 기준
A4_WIDTH_MM = 210
A4_HEIGHT_MM = 297

class PDF(FPDF):
    def footer(self):
        self.set_y(-15)
        self.set_font('Arial', 'I', 8)
        self.cell(0, 10, f'Chessboard Pattern ({SQUARE_SIZE_MM}mm)', 0, 0, 'C')

def create_chessboard():
    pdf = PDF(orientation='L', unit='mm', format='A4') # 가로(Landscape) 모드
    pdf.add_page()
    
    # 중앙 정렬을 위한 시작점 계산
    board_width = COLS * SQUARE_SIZE_MM
    board_height = ROWS * SQUARE_SIZE_MM
    
    start_x = (297 - board_width) / 2  # A4 가로 길이 297mm
    start_y = (210 - board_height) / 2 # A4 세로 길이 210mm
    
    print(f"🏁 체커보드 생성 시작: {COLS}x{ROWS} (격자 크기: {SQUARE_SIZE_MM}mm)")

    # 체커보드 그리기
    pdf.set_fill_color(0, 0, 0) # 검은색

    for r in range(ROWS):
        for c in range(COLS):
            # (홀수 행, 짝수 열) 또는 (짝수 행, 홀수 열)일 때 검은색 칠하기
            if (r + c) % 2 == 1:
                x = start_x + (c * SQUARE_SIZE_MM)
                y = start_y + (r * SQUARE_SIZE_MM)
                pdf.rect(x, y, SQUARE_SIZE_MM, SQUARE_SIZE_MM, 'F') # 'F' = Fill

    # 텍스트 안내 (사용자 편의)
    pdf.set_font("Arial", size=10)
    pdf.text(10, 200, f"Pattern Size: {COLS}x{ROWS} Squares")
    pdf.text(10, 205, f"Square Side: {SQUARE_SIZE_MM}mm (Check with ruler!)")
    pdf.text(10, 210, f"Internal Corners: {COLS-1}x{ROWS-1} (Input this to OpenCV)")

    pdf.output(OUTPUT_FILENAME)
    print(f"✅ '{OUTPUT_FILENAME}' 생성 완료!")

if __name__ == "__main__":
    create_chessboard()