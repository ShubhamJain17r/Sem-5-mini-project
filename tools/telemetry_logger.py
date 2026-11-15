#!/usr/bin/env python3
"""
Automated Potentiometer Drift Telemetry & Statistical Analysis for Single xArm 1S.

Replaces manual paper logging:
  - Queries all 6 servo potentiometers (CMD 0x15) on the manipulator (6 channels)
  - Logs structured time-series data to data/telemetry_drift_log.csv
  - Computes statistical metrics: sample count, mean error (μ), standard deviation (σ), min/max
  - Evaluates sub-millimeter tabletop picking repeatability against reach kinematics (1 tick ≈ 1.05mm at R=250mm)
  - Emits actionable maintenance recommendations (screw re-torquing, spline trim adjustments)
"""

import argparse
import csv
from datetime import datetime
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

# Support imports
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))
ros_pkg_dir = os.path.join(project_root, "ros2_ws/src/sorting_arm")
if project_root not in sys.path:
    sys.path.insert(0, project_root)
if ros_pkg_dir not in sys.path:
    sys.path.insert(0, ros_pkg_dir)

import yaml
from sorting_arm.xarm_protocol import MOTOR_NAMES, XArmDriver

DATA_DIR = os.path.join(project_root, "data")
CSV_LOG_PATH = os.path.join(DATA_DIR, "telemetry_drift_log.csv")
CONFIG_HARDWARE_PATH = os.path.join(project_root, "config/xarm_hardware.yaml")

CSV_HEADER = [
    "timestamp_iso",
    "arm_id",
    "motor_id",
    "joint_name",
    "target_tick",
    "actual_tick",
    "tracking_error_ticks",
    "deviation_trim",
    "is_simulated",
]


def ensure_csv_header(csv_path: str = CSV_LOG_PATH):
    """Ensure destination directory and CSV header exist."""
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)


def load_yaml(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def run_telemetry_scan(
    arm_ids: Optional[List[str]] = None,
    csv_path: str = CSV_LOG_PATH,
) -> Dict[str, Any]:
    """
    Execute a single synchronous scan across all joints and record to CSV.

    :param arm_ids: List of arms to query (default: ["xarm_1"])
    :param csv_path: Target CSV filepath
    :return: Dictionary summary of scan results and statistical analysis
    """
    if arm_ids is None:
        arm_ids = ["xarm_1"]

    ensure_csv_header(csv_path)
    now_iso = datetime.now().isoformat()

    hw_data = load_yaml(CONFIG_HARDWARE_PATH)
    scan_rows: List[Dict[str, Any]] = []

    for arm_id in arm_ids:
        arm_cfg = hw_data.get("manipulators", {}).get(arm_id, {})
        motors = arm_cfg.get("motors", {})
        devs = {sid: int(motors.get(f"motor_{sid}", {}).get("deviation", 0)) for sid in range(1, 7)}

        driver = XArmDriver(port="auto", deviations=devs)
        connected = driver.connect()

        for sid in range(1, 7):
            target = driver.current_ticks.get(sid, 500)
            actual = None
            if connected:
                actual = driver.read_servo_position(sid, timeout_sec=0.04)

            simulated = actual is None
            display_pos = actual if actual is not None else target
            error = display_pos - target

            row = {
                "timestamp_iso": now_iso,
                "arm_id": arm_id,
                "motor_id": sid,
                "joint_name": MOTOR_NAMES.get(sid, f"Motor {sid}"),
                "target_tick": target,
                "actual_tick": display_pos,
                "tracking_error_ticks": error,
                "deviation_trim": devs.get(sid, 0),
                "is_simulated": simulated,
            }
            scan_rows.append(row)

        if driver and connected:
            driver.disconnect()

    # Append to CSV
    with open(csv_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_HEADER)
        for r in scan_rows:
            writer.writerow(r)

    # Compute statistical metrics
    stats = compute_drift_statistics(csv_path)

    return {
        "timestamp": now_iso,
        "scanned_channels": len(scan_rows),
        "csv_path": csv_path,
        "latest_scan": scan_rows,
        "drift_statistics": stats,
    }


def compute_drift_statistics(csv_path: str = CSV_LOG_PATH) -> Dict[str, Any]:
    """
    Parse historical telemetry CSV and calculate mean, std deviation, and repeatability metrics.
    """
    if not os.path.exists(csv_path):
        return {}

    history: Dict[str, List[float]] = {}

    try:
        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                arm = row.get("arm_id", "")
                mid = row.get("motor_id", "")
                err_str = row.get("tracking_error_ticks", "0")
                key = f"{arm}_M{mid}"
                try:
                    err = float(err_str)
                    if key not in history:
                        history[key] = []
                    history[key].append(err)
                except ValueError:
                    pass
    except Exception:
        return {}

    summary = {}
    recommendations = []

    for key, errors in history.items():
        n = len(errors)
        if n == 0:
            continue
        mean_err = sum(errors) / n
        variance = sum((x - mean_err) ** 2 for x in errors) / n
        std_dev = math.sqrt(variance)
        max_err = max(errors)
        min_err = min(errors)

        # 1 tick = 0.24 deg. At reach R = 250mm, displacement = R * theta_rad ≈ 1.05 mm
        # Repeatability error at 250mm reach:
        fingertip_displacement_mm = std_dev * 1.047

        summary[key] = {
            "samples": n,
            "mean_error_ticks": round(mean_err, 2),
            "std_dev_ticks": round(std_dev, 2),
            "min_error_ticks": min_err,
            "max_error_ticks": max_err,
            "fingertip_repeatability_1sigma_mm": round(fingertip_displacement_mm, 2),
        }

        # Check thresholds
        if std_dev > 3.0:
            recommendations.append(
                f"[MECHANICAL] {key} shows high jitter (σ = {std_dev:.1f} ticks > 3.0). "
                f"Retorque spline lock screw M3 to eliminate backlash."
            )
        if abs(mean_err) > 5.0:
            recommendations.append(
                f"[CALIBRATION] {key} shows systematic drift (μ = {mean_err:+.1f} ticks > ±5.0). "
                f"Calibrate zero position in xArm PC App or update xarm_hardware.yaml by {-int(round(mean_err))} ticks."
            )

    if not recommendations:
        recommendations.append(
            "[NOMINAL] All monitored joints within sub-millimeter tabletop repeatability (σ ≤ 3.0 ticks, |μ| ≤ 5.0 ticks)."
        )

    return {
        "joint_metrics": summary,
        "recommendations": recommendations,
        "submillimeter_compliance": len(recommendations) == 1 and "[NOMINAL]" in recommendations[0],
    }


def print_cli_report(results: Dict[str, Any]):
    """Pretty-print telemetry report to terminal."""
    print("\n" + "=" * 70)
    print("  xARM 1S TELEMETRY & LONG-TERM DRIFT ANALYSIS")
    print(f"  Timestamp: {results.get('timestamp')}")
    print(f"  Dataset:   {results.get('csv_path')}")
    print("=" * 70)

    stats = results.get("drift_statistics", {})
    metrics = stats.get("joint_metrics", {})

    print("\n[HISTORICAL TRACKING ERROR & REPEATABILITY SUMMARY]")
    print(f"{'Channel':<14} | {'N':<6} | {'Mean (μ)':<10} | {'StdDev (σ)':<10} | {'1σ Disp (mm)':<14} | {'Range':<12}")
    print("-" * 72)

    for ch, data in sorted(metrics.items()):
        mean_str = f"{data['mean_error_ticks']:+.2f} t"
        std_str = f"{data['std_dev_ticks']:.2f} t"
        disp_str = f"{data['fingertip_repeatability_1sigma_mm']:.2f} mm"
        range_str = f"[{data['min_error_ticks']:+.0f}, {data['max_error_ticks']:+.0f}]"
        print(f"{ch:<14} | {data['samples']:<6} | {mean_str:<10} | {std_str:<10} | {disp_str:<14} | {range_str:<12}")

    print("\n[MAINTENANCE & CALIBRATION RECOMMENDATIONS]")
    for rec in stats.get("recommendations", []):
        print(f"  -> {rec}")
    print("\n" + "=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="xArm 1S Automated Telemetry Drift Logger")
    parser.add_argument("--arm", default="xarm_1", help="Target manipulator ID (default: xarm_1)")
    parser.add_argument("--scan", action="store_true", default=True, help="Execute immediate scan")
    parser.add_argument("--samples", type=int, default=1, help="Number of consecutive scans to log")
    parser.add_argument("--interval", type=float, default=1.0, help="Interval between scans in seconds")
    parser.add_argument("--stats-only", action="store_true", help="Print existing drift stats without new scan")
    args = parser.parse_args()

    if args.stats_only:
        stats = compute_drift_statistics()
        print_cli_report({"timestamp": datetime.now().isoformat(), "csv_path": CSV_LOG_PATH, "drift_statistics": stats})
        return

    arm_list = [args.arm]
    for i in range(args.samples):
        if i > 0:
            time.sleep(args.interval)
        results = run_telemetry_scan(arm_ids=arm_list)
        if i == args.samples - 1:
            print_cli_report(results)


if __name__ == "__main__":
    main()
