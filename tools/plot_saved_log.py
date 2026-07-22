import pandas as pd
import matplotlib.pyplot as plt
import os
from datetime import datetime

# =========================
# 설정 부분
# =========================
LOG_FILE = "accelerometer_log.txt"  # Tera Term에서 저장한 로그 파일명

os.makedirs("accel_graphs", exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
speed_png = f"accel_graphs/speed_graph_{timestamp}.png"
accel_png = f"accel_graphs/accel_raw_graph_{timestamp}.png"
clean_csv = f"accel_graphs/cleaned_accel_log_{timestamp}.csv"

rows = []

with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        line = line.strip()

        if not line.startswith("CSV,"):
            continue

        parts = line.split(",")

        # CSV,time_ms,speed_cms,turn_count,front,right,left,rear,ax,ay,az
        if len(parts) != 11:
            continue

        rows.append(parts)

if len(rows) == 0:
    print("No CSV data found.")
    print("Check that your log contains lines starting with 'CSV,'.")
    exit()

columns = [
    "type", "time_ms", "speed_cms", "turn_count",
    "front_cm", "right_cm", "left_cm", "rear_us_cm",
    "ax_mg", "ay_mg", "az_mg"
]

df = pd.DataFrame(rows, columns=columns)

for col in columns[1:]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.dropna()

df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0

# 정리된 CSV 저장
df.to_csv(clean_csv, index=False)

# =========================
# Speed graph
# =========================
plt.figure()
plt.plot(df["time_s"], df["speed_cms"])
plt.xlabel("Time (s)")
plt.ylabel("Speed (cm/s)")
plt.title("Estimated Vehicle Speed from Accelerometer")
plt.grid(True)
plt.savefig(speed_png, dpi=200, bbox_inches="tight")
plt.show()

# =========================
# Raw accelerometer graph
# =========================
plt.figure()
plt.plot(df["time_s"], df["ax_mg"], label="ax_mg")
plt.plot(df["time_s"], df["ay_mg"], label="ay_mg")
plt.plot(df["time_s"], df["az_mg"], label="az_mg")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (mg)")
plt.title("Accelerometer Raw Data")
plt.legend()
plt.grid(True)
plt.savefig(accel_png, dpi=200, bbox_inches="tight")
plt.show()

print(f"Saved cleaned CSV to: {clean_csv}")
print(f"Saved speed graph to: {speed_png}")
print(f"Saved accelerometer raw graph to: {accel_png}")