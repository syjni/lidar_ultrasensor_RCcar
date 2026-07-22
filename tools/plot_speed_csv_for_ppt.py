#!/usr/bin/env python3
"""
Plot Maze Runner accelerometer/speed CSV logs for PPT.

Input log lines must contain rows like:
CSV,time_ms,speed_cms,turn_count,front_cm,right_cm,left_cm,rear_us_cm,ax_mg,ay_mg,az_mg

Usage:
  python plot_speed_csv_for_ppt.py teraterm_log.txt
  python plot_speed_csv_for_ppt.py teraterm_log.txt --outdir graphs

Outputs:
  speed_graph_for_ppt.png
  accel_xyz_graph_for_ppt.png
  distance_graph_for_ppt.png
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

COLUMNS = [
    "tag", "time_ms", "speed_cms", "turn_count",
    "front_cm", "right_cm", "left_cm", "rear_us_cm",
    "ax_mg", "ay_mg", "az_mg",
]

UNKNOWN_CM = 999


def parse_log(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line.startswith("CSV,"):
                continue
            parts = next(csv.reader([line]))
            if len(parts) < len(COLUMNS):
                continue
            try:
                record = {name: float(value) for name, value in zip(COLUMNS[1:], parts[1:])}
            except ValueError:
                continue
            rows.append(record)
    if not rows:
        raise SystemExit(f"No CSV rows found in {path}")

    t0 = rows[0]["time_ms"]
    for r in rows:
        r["time_s"] = (r["time_ms"] - t0) / 1000.0
        for key in ("front_cm", "right_cm", "left_cm", "rear_us_cm"):
            if r[key] >= UNKNOWN_CM:
                r[key] = math.nan
    return rows


def series(rows: list[dict[str, float]], key: str) -> list[float]:
    return [r[key] for r in rows]


def save_speed_graph(rows: list[dict[str, float]], outpath: Path) -> None:
    t = series(rows, "time_s")
    speed = series(rows, "speed_cms")
    turns = series(rows, "turn_count")

    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=180)
    ax.plot(t, speed, linewidth=2.2, label="Estimated speed")
    ax.set_title("Estimated speed from LIS2DH12 accelerometer")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (cm/s)")
    ax.grid(True, alpha=0.3)

    # Mark turn-count changes lightly so the graph is useful for the 5-turn requirement.
    last_turn = turns[0]
    for ti, turn in zip(t, turns):
        if turn != last_turn:
            ax.axvline(ti, linestyle="--", linewidth=0.9, alpha=0.45)
            last_turn = turn

    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(outpath, bbox_inches="tight")
    plt.close(fig)


def save_accel_graph(rows: list[dict[str, float]], outpath: Path) -> None:
    t = series(rows, "time_s")
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=180)
    ax.plot(t, series(rows, "ax_mg"), linewidth=1.8, label="X acceleration")
    ax.plot(t, series(rows, "ay_mg"), linewidth=1.4, label="Y acceleration")
    ax.plot(t, series(rows, "az_mg"), linewidth=1.4, label="Z acceleration")
    ax.set_title("LIS2DH12 acceleration samples")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (mg)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(outpath, bbox_inches="tight")
    plt.close(fig)


def save_distance_graph(rows: list[dict[str, float]], outpath: Path) -> None:
    t = series(rows, "time_s")
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=180)
    ax.plot(t, series(rows, "front_cm"), linewidth=2.0, label="Front LiDAR")
    ax.plot(t, series(rows, "right_cm"), linewidth=1.5, label="Right LiDAR")
    ax.plot(t, series(rows, "left_cm"), linewidth=1.5, label="Left LiDAR")
    ax.plot(t, series(rows, "rear_us_cm"), linewidth=1.5, label="Rear ultrasonic")
    ax.set_title("Distance sensors during autonomous driving")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (cm)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(outpath, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", type=Path, help="Tera Term log file containing CSV lines")
    parser.add_argument("--outdir", type=Path, default=Path("."), help="Output directory")
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)
    rows = parse_log(args.logfile)

    speed_path = args.outdir / "speed_graph_for_ppt.png"
    accel_path = args.outdir / "accel_xyz_graph_for_ppt.png"
    distance_path = args.outdir / "distance_graph_for_ppt.png"

    save_speed_graph(rows, speed_path)
    save_accel_graph(rows, accel_path)
    save_distance_graph(rows, distance_path)

    print(f"Parsed {len(rows)} CSV rows")
    print(f"Saved: {speed_path}")
    print(f"Saved: {accel_path}")
    print(f"Saved: {distance_path}")


if __name__ == "__main__":
    main()
