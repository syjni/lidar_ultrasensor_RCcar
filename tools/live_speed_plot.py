import serial
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime
import csv
import os
import sys
import time

# =====================================================
# 설정 부분
# =====================================================
# 기본값: Bluetooth HC-06 기준
# USB로 테스트할 때는 PORT를 Arduino USB COM 번호로, BAUD를 115200으로 바꾸세요.
PORT = "COM10"      # 예: USB는 "COM5", Bluetooth는 "COM7" 등
BAUD = 9600        # USB UART: 115200 / Bluetooth HC-06: 9600

WINDOW_POINTS = 300       # 실시간 화면에 보여줄 최근 데이터 개수
STATUS_INTERVAL_SEC = 2.0 # CSV가 안 들어올 때 상태 출력 주기

# 터미널에서 인자로도 지정 가능:
#   python live_speed_plot_fixed.py COM5 115200
#   python live_speed_plot_fixed.py COM7 9600
if len(sys.argv) >= 2:
    PORT = sys.argv[1]
if len(sys.argv) >= 3:
    BAUD = int(sys.argv[2])

# =====================================================
# 저장 폴더 준비
# =====================================================
os.makedirs("accel_logs", exist_ok=True)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"accel_logs/live_capture_{timestamp}.csv"
png_filename = f"accel_logs/live_speed_graph_{timestamp}.png"

# =====================================================
# 유틸 함수
# =====================================================
def safe_int(s, default=0):
    try:
        return int(float(s))  # "12.3"이나 "999" 모두 허용
    except (ValueError, TypeError):
        return default


def print_port_help():
    print("\nCheck these:")
    print("1. Is the COM port number correct?")
    print("2. Is Tera Term closed? Python and Tera Term cannot open the same COM port at once.")
    print("3. Is the baud rate correct? USB=115200, Bluetooth=9600")
    print("4. Does the firmware output lines starting with 'CSV,'?")

# =====================================================
# 시리얼 연결
# =====================================================
print(f"Opening serial port {PORT} at {BAUD} bps...")

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
except serial.SerialException as e:
    print("Serial port open failed.")
    print(e)
    print_port_help()
    raise SystemExit

# =====================================================
# 데이터 구조
# =====================================================
# 실시간 표시용: 최근 구간만 보여줌
view_time = deque(maxlen=WINDOW_POINTS)
view_speed = deque(maxlen=WINDOW_POINTS)

# 최종 그래프용: 전체 누적
all_time = []
all_speed = []
all_turn = []

start_time_ms = None
last_turn = None
turn_boundaries = []   # (time_s, turn_count)

raw_line_count = 0
csv_line_count = 0
bad_csv_count = 0
last_status_time = time.time()
shown_non_csv_examples = 0

# =====================================================
# CSV 저장 파일 열기
# =====================================================
csv_file = open(csv_filename, "w", newline="", encoding="utf-8")
writer = csv.writer(csv_file)
writer.writerow([
    "time_ms", "time_s", "speed_cms", "turn_count",
    "front_cm", "right_cm", "left_cm", "rear_us_cm",
    "ax_mg", "ay_mg", "az_mg"
])

# =====================================================
# 실시간 그래프 설정
# =====================================================
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=1.5)
ax.set_title("Estimated Vehicle Speed from Accelerometer")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (cm/s)")
ax.grid(True)
plt.show(block=False)

print("Reading data... Press Ctrl+C to stop.")
print(f"Saving CSV to: {csv_filename}")
print("Waiting for lines that start with 'CSV,' ...")

# =====================================================
# 실시간 읽기 + 그래프 업데이트
# =====================================================
try:
    while True:
        raw = ser.readline().decode(errors="ignore").strip()

        # 데이터가 없어도 GUI가 멈춘 것처럼 보이지 않도록 pause 수행
        if raw == "":
            now = time.time()
            if now - last_status_time >= STATUS_INTERVAL_SEC:
                print(f"Waiting... raw lines={raw_line_count}, CSV lines={csv_line_count}, bad CSV={bad_csv_count}")
                last_status_time = now
            plt.pause(0.01)
            continue

        raw_line_count += 1

        # 코드가 '# FIVE_TURNS_DONE...' 같은 marker를 출력하면 표시만 해줌
        if raw.startswith("#"):
            print(raw)
            plt.pause(0.01)
            continue

        if not raw.startswith("CSV,"):
            # 처음 몇 줄만 보여줘서 baud/출력 형식 확인에 도움
            if shown_non_csv_examples < 5:
                print(f"Non-CSV line: {raw}")
                shown_non_csv_examples += 1
            plt.pause(0.01)
            continue

        parts = raw.split(",")
        # CSV,time_ms,speed_cms,turn,front,right,left,rear,ax,ay,az
        if len(parts) < 11:
            bad_csv_count += 1
            print(f"Bad CSV length ({len(parts)}): {raw}")
            plt.pause(0.01)
            continue

        time_ms   = safe_int(parts[1])
        speed_cms = safe_int(parts[2])
        turn_cnt  = safe_int(parts[3])
        front_cm  = safe_int(parts[4])
        right_cm  = safe_int(parts[5])
        left_cm   = safe_int(parts[6])
        rear_cm   = safe_int(parts[7])
        ax_mg     = safe_int(parts[8])
        ay_mg     = safe_int(parts[9])
        az_mg     = safe_int(parts[10])

        if start_time_ms is None:
            start_time_ms = time_ms
            print("First CSV received! Graph should start moving now.")

        time_s = (time_ms - start_time_ms) / 1000.0
        csv_line_count += 1

        # CSV 백업 저장
        writer.writerow([
            time_ms, time_s, speed_cms, turn_cnt,
            front_cm, right_cm, left_cm, rear_cm,
            ax_mg, ay_mg, az_mg
        ])
        csv_file.flush()

        # 전체 누적
        all_time.append(time_s)
        all_speed.append(speed_cms)
        all_turn.append(turn_cnt)

        # turn 경계 감지
        if last_turn is not None and turn_cnt != last_turn:
            turn_boundaries.append((time_s, turn_cnt))
        last_turn = turn_cnt

        # 실시간 표시: 최근 구간만
        view_time.append(time_s)
        view_speed.append(speed_cms)
        line.set_data(list(view_time), list(view_speed))
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw_idle()
        plt.pause(0.01)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    print("Closing serial port...")
    try:
        ser.close()
    except Exception:
        pass
    csv_file.close()

    if len(all_time) == 0:
        print("No CSV data received. Final graph was not saved.")
        print(f"Empty CSV header file may exist at: {csv_filename}")
        print_port_help()
    else:
        print("Drawing full-run graph...")
        plt.ioff()
        fig2, ax2 = plt.subplots(figsize=(12, 5))
        ax2.plot(all_time, all_speed, lw=1.2, label="Speed (cm/s)")
        ax2.set_title("Vehicle Speed over 5 Turns")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Speed (cm/s)")
        ax2.grid(True)

        # turn 경계마다 세로선 + 라벨
        y_min, y_max = ax2.get_ylim()
        label_y = y_max - (y_max - y_min) * 0.05
        for t_s, tc in turn_boundaries:
            ax2.axvline(t_s, linestyle="--", alpha=0.5)
            ax2.text(t_s, label_y, f"turn {tc}", rotation=90, va="top", fontsize=8)

        ax2.legend()
        fig2.savefig(png_filename, dpi=200, bbox_inches="tight")
        print(f"Saved CSV: {csv_filename}")
        print(f"Saved graph: {png_filename}")
        plt.show()
