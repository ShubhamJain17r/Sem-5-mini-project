#!/usr/bin/env python3
"""
xArm 1S Kinematic Grid Traversal & Perception Ground-Truth Survey.

Systematically traverses all 42 arena coordinates:
  Rows 6 to 11 (Y = 275.0 mm down to 25.0 mm, 50mm spacing)
  Cols 0 to 6  (X = 25.0 mm up to 325.0 mm, 50mm spacing)

Features:
  - Records continuous live video from webcam (/dev/video4) with real-time HUD telemetry.
  - Commands analytical inverse kinematics to each coordinate at safe hover altitude (Z = 35mm).
  - Handles reach envelope boundaries (R < 120mm near-base and R > 330mm far reach).
  - Reads actual potentiometer ticks from hardware and calculates Forward Kinematics (FK).
  - Logs position accuracy, tracking errors, and joint angles into CSV.
  - Automatically compresses video to artifacts/arena_grid_traversal.mp4 via ffmpeg.
  - Generates 2D kinematic reachability and accuracy analysis plot.
"""

import argparse
import math
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

# Ensure repository root and sorting_arm package are in sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, ".."))
sorting_arm_pkg = os.path.join(repo_root, "ros2_ws", "src", "sorting_arm")
for p in [repo_root, sorting_arm_pkg]:
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
from sorting_arm.kinematics import (
    DEFAULT_PARAMS,
    KinematicParameters,
    solve_fk,
    load_kinematic_parameters,
    solve_ik,
)
from sorting_arm.xarm_protocol import XArmDriver, find_all_xarm_devices
import yaml


def polar_to_world(r: float, theta_arm_deg: float, base_x=175.0, base_y=375.0, heading_deg=-90.0) -> Tuple[float, float]:
    """Converts arm polar coordinates (r, theta) to world millimeter coordinates (Xw, Yw)."""
    th_world_rad = math.radians(heading_deg + theta_arm_deg)
    xw = base_x + r * math.cos(th_world_rad)
    yw = base_y + r * math.sin(th_world_rad)
    return xw, yw


def world_to_polar(xw: float, yw: float, base_x=175.0, base_y=375.0, heading_deg=-90.0) -> Tuple[float, float]:
    """Converts world millimeter coordinates (Xw, Yw) to arm polar coordinates (r, theta_arm)."""
    dx = xw - base_x
    dy = yw - base_y
    r = math.hypot(dx, dy)
    theta_world = math.degrees(math.atan2(dy, dx))
    theta_arm = theta_world - heading_deg
    while theta_arm > 180.0:
        theta_arm -= 360.0
    while theta_arm < -180.0:
        theta_arm += 360.0
    return r, theta_arm


def draw_hud(
    frame: np.ndarray,
    target_row: int,
    target_col: int,
    target_xw: float,
    target_yw: float,
    target_zw: float,
    target_r: float,
    target_theta: float,
    actual_xw: float,
    actual_yw: float,
    actual_zw: float,
    actual_r: float,
    actual_theta: float,
    status: str,
    target_ticks: Dict[int, int],
    actual_ticks: Dict[int, int],
    is_reachable: bool,
) -> np.ndarray:
    """Renders informative live telemetry HUD overlay onto video frame."""
    hud = frame.copy()
    h, w, _ = frame.shape

    # Semi-transparent top banner
    overlay = hud.copy()
    cv2.rectangle(overlay, (0, 0), (w, 115), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.75, hud, 0.25, 0, hud)

    # Title
    cv2.putText(hud, "xArm 1S Kinematic Arena Grid Survey", (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    
    # Status badge
    badge_color = (0, 200, 0) if is_reachable else (0, 140, 255)
    cv2.rectangle(hud, (w - 180, 8), (w - 10, 32), badge_color, -1)
    badge_text = "REACHABLE" if is_reachable else "OUT OF REACH"
    cv2.putText(hud, badge_text, (w - 170, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)

    # Target info
    t_text = f"Target: Grid({target_row}, {target_col}) | World: X={target_xw:5.1f} Y={target_yw:5.1f} Z={target_zw:4.1f} mm | R={target_r:5.1f}mm Th={target_theta:+5.1f}deg"
    cv2.putText(hud, t_text, (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # Actual feedback info
    err_r = actual_r - target_r
    err_th = actual_theta - target_theta
    a_text = f"Actual: World: X={actual_xw:5.1f} Y={actual_yw:5.1f} Z={actual_zw:4.1f} mm | R={actual_r:5.1f}mm Th={actual_theta:+5.1f}deg | Err: dR={err_r:+4.1f}mm dTh={err_th:+4.1f}deg"
    cv2.putText(hud, a_text, (15, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 255, 100), 1)

    # Servo ticks telemetry
    cmd_str = " ".join(f"M{i}:{target_ticks.get(i, 0)}" for i in range(1, 7))
    act_str = " ".join(f"M{i}:{actual_ticks.get(i, 0)}" for i in range(1, 7))
    cv2.putText(hud, f"Ticks Cmd: {cmd_str} | Act: {act_str}", (15, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

    return hud


def run_survey(
    device_path: str = "/dev/video4",
    arm_port: str = "auto",
    config_dir: str = "config",
    safe_z_mm: float = 35.0,
    hold_time_sec: float = 0.5,
    output_video_path: str = "artifacts/arena_grid_traversal.mp4",
    output_csv_path: str = "data/grid_traversal_telemetry.csv",
    dry_run: bool = False,
):
    print("========================================================================")
    print("  xArm 1S Complete Arena Grid Traversal & Kinematics Survey")
    print(f"  Camera Device: {device_path} | Serial Port: {arm_port} | Dry Run: {dry_run}")
    print(f"  Hover Altitude: Z = {safe_z_mm:.1f} mm | Hold Time: {hold_time_sec:.1f} s")
    print("========================================================================")

    # 1. Load Hardware & Kinematic Parameters
    hw_yaml = os.path.join(config_dir, "xarm_hardware.yaml")
    params = load_kinematic_parameters(hw_yaml, arm_id="xarm_1")
    base_x = 175.0
    base_y = 375.0
    nominal_heading = -90.0

    # 2. Connect to Manipulator
    driver = None
    if dry_run:
        print("[Hardware] Running in DRY-RUN mode (Simulated xArm hardware)...")
        class MockDriver:
            def __init__(self):
                self.current_ticks = {1: 500, 2: 500, 3: 500, 4: 500, 5: 500, 6: 500}
            def send_multi_servo_move(self, ticks, time_ms=1000):
                self.current_ticks.update(ticks)
            def read_all_servo_positions(self, timeout_sec=0.05):
                return dict(self.current_ticks)
            def read_servo_position(self, sid, timeout_sec=0.05):
                return self.current_ticks.get(sid, 500)
            def disconnect(self):
                pass
        driver = MockDriver()
    else:
        target_port = arm_port
        if target_port in ["auto", "", None]:
            devs = find_all_xarm_devices()
            if devs:
                target_port = devs[0][0]
            else:
                target_port = "/dev/hidraw3"
        print(f"[Hardware] Connecting to xArm on {target_port}...")
        driver = XArmDriver(
            port=target_port,
            min_ticks=params.min_safe_ticks,
            max_ticks=params.max_safe_ticks,
        )
        if not driver.connect():
            print(f"[Error] Failed to connect to xArm on {target_port}.")
            print("Please grant permissions:")
            print("  sudo chmod 666 /dev/hidraw*")
            print("Or install the permanent udev rule:")
            print("  echo 'SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5750\", MODE=\"0666\", GROUP=\"plugdev\"' | sudo tee /etc/udev/rules.d/99-xarm.rules && sudo udevadm control --reload-rules && sudo udevadm trigger")
            return False
        print("[Hardware] Connected successfully!")

    # 3. Open Camera Capture & Video Writer
    print(f"[Vision] Opening camera stream {device_path}...")
    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        print(f"[Error] Failed to open camera on {device_path}.")
        driver.disconnect()
        return False

    # Discard unexposed buffer frames
    for _ in range(10):
        cap.read()

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
    fps = 15.0

    raw_avi_path = "/tmp/arena_survey_raw.avi"
    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    video_writer = cv2.VideoWriter(raw_avi_path, fourcc, fps, (width, height))

    # Helper function to capture and record frames during motion
    last_hud_params = {}

    def record_frames_duration(duration_sec: float):
        n_frames = max(1, int(round(duration_sec * fps)))
        for _ in range(n_frames):
            ret, frame = cap.read()
            if ret and frame is not None:
                if last_hud_params:
                    hud_frame = draw_hud(frame, **last_hud_params)
                else:
                    hud_frame = frame
                video_writer.write(hud_frame)
            time.sleep(1.0 / fps)

    # 4. Prepare Telemetry CSV
    os.makedirs(os.path.dirname(os.path.abspath(output_csv_path)), exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(output_video_path)), exist_ok=True)
    csv_file = open(output_csv_path, "w")
    csv_file.write(
        "timestamp_iso,target_row,target_col,target_xw,target_yw,target_zw,target_r,target_theta,"
        "cmd_xw,cmd_yw,cmd_zw,cmd_r,cmd_theta,is_reachable,status,"
        "actual_xw,actual_yw,actual_zw,actual_r,actual_theta,error_r_mm,error_theta_deg,"
        "cmd_m1,cmd_m2,cmd_m3,cmd_m4,cmd_m5,cmd_m6,"
        "act_m1,act_m2,act_m3,act_m4,act_m5,act_m6\n"
    )

    # 5. Move to Home Pose First
    print("[Traversal] Moving arm to Upright Home Pose...")
    home_ticks = {1: 500, 2: 500, 3: 500, 4: 496, 5: 501, 6: 502}
    driver.send_multi_servo_move(home_ticks, time_ms=1600)
    record_frames_duration(1.8)

    survey_data = []

    # 6. Grid Traversal Loop
    try:
        # Loop through rows 6 to 11
        for row in range(6, 12):
            target_yw = 275.0 - (row - 6) * 50.0
            print(f"\n=======================================================")
            print(f"  TRAVERSING ROW {row} (Ground Truth Y = {target_yw:.1f} mm)")
            print(f"=======================================================")

            # Standard left->right traversal: cols 0 to 6 on every row
            cols = list(range(0, 7))

            for col in cols:
                target_xw = 25.0 + col * 50.0
                target_zw = safe_z_mm

                # Compute true polar coordinates relative to base mount
                r_target, theta_target = world_to_polar(target_xw, target_yw, base_x, base_y, nominal_heading)

                # Clamp/check kinematic feasibility
                is_reachable = True
                status = "REACHABLE"
                cmd_r = r_target
                cmd_th = theta_target
                cmd_zw = target_zw

                # Inner near-base reach constraint
                if r_target < 130.0:
                    cmd_r = 130.0
                    is_reachable = False
                    status = "NEAR_BASE_CLAMP"
                # Outer reach limit constraint
                elif r_target > 325.0:
                    cmd_r = 320.0
                    is_reachable = False
                    status = "OUTER_REACH_CLAMP"

                # Solve Inverse Kinematics
                # Motor 1 (claw closed like a probe tip: tick 750)
                ik_sol = solve_ik(
                    r=cmd_r,
                    theta=cmd_th,
                    z=cmd_zw,
                    claw_tick=750,
                    roll_tick=500,
                    params=params,
                )

                if not ik_sol.success:
                    is_reachable = False
                    status = "IK_INFEASIBLE"
                    print(f"  [Skip] Grid({row}, {col}) IK Unsolvable: {ik_sol.message}")
                    continue

                cmd_ticks = ik_sol.ticks

                # Recompute commanded world Cartesian coordinates
                cmd_xw, cmd_yw = polar_to_world(cmd_r, cmd_th, base_x, base_y, nominal_heading)

                # Send motion command to servos (extra time for long sweep to col 0)
                move_ms = 1400 if (col == 0 and row > 6) else 950
                driver.send_multi_servo_move(cmd_ticks, time_ms=move_ms)
                
                # Update HUD parameters during motion
                last_hud_params = {
                    "target_row": row,
                    "target_col": col,
                    "target_xw": target_xw,
                    "target_yw": target_yw,
                    "target_zw": target_zw,
                    "target_r": r_target,
                    "target_theta": theta_target,
                    "actual_xw": cmd_xw,
                    "actual_yw": cmd_yw,
                    "actual_zw": cmd_zw,
                    "actual_r": cmd_r,
                    "actual_theta": cmd_th,
                    "status": status,
                    "target_ticks": cmd_ticks,
                    "actual_ticks": cmd_ticks,
                    "is_reachable": is_reachable,
                }

                # Record frames during transit
                record_frames_duration(move_ms / 1000.0)

                # Read actual telemetry feedback from hardware
                raw_positions = driver.read_all_servo_positions(timeout_sec=0.04)
                actual_ticks = {
                    sid: raw_positions.get(sid) if raw_positions.get(sid) is not None else cmd_ticks.get(sid, 500)
                    for sid in range(1, 7)
                }

                # Compute Forward Kinematics (FK) from actual feedback ticks
                fk_pose = solve_fk(
                    tick_6=actual_ticks.get(6, 500),
                    tick_5=actual_ticks.get(5, 500),
                    tick_4=actual_ticks.get(4, 500),
                    tick_3=actual_ticks.get(3, 500),
                    params=params,
                    use_claw_tip=True,
                )
                act_xw, act_yw = polar_to_world(fk_pose.r, fk_pose.theta, base_x, base_y, nominal_heading)
                act_zw = fk_pose.z
                err_r = fk_pose.r - r_target
                err_th = fk_pose.theta - theta_target

                # Update HUD with real measured feedback
                last_hud_params["actual_xw"] = act_xw
                last_hud_params["actual_yw"] = act_yw
                last_hud_params["actual_zw"] = act_zw
                last_hud_params["actual_r"] = fk_pose.r
                last_hud_params["actual_theta"] = fk_pose.theta
                last_hud_params["actual_ticks"] = actual_ticks

                # Record frames during dwell / hold time
                record_frames_duration(hold_time_sec)

                iso_time = time.strftime("%Y-%m-%dT%H:%M:%S")
                print(
                    f"  Grid({row:2d}, {col:2d}): Target=({target_xw:5.1f}, {target_yw:5.1f})mm | "
                    f"Reached=({act_xw:5.1f}, {act_yw:5.1f})mm | errR={err_r:+4.1f}mm errTh={err_th:+4.1f}° | [{status}]"
                )

                # Write record to CSV
                csv_file.write(
                    f"{iso_time},{row},{col},{target_xw:.1f},{target_yw:.1f},{target_zw:.1f},{r_target:.1f},{theta_target:.1f},"
                    f"{cmd_xw:.1f},{cmd_yw:.1f},{cmd_zw:.1f},{cmd_r:.1f},{cmd_th:.1f},{is_reachable},{status},"
                    f"{act_xw:.1f},{act_yw:.1f},{act_zw:.1f},{fk_pose.r:.1f},{fk_pose.theta:.1f},{err_r:.1f},{err_th:.1f},"
                    f"{cmd_ticks.get(1, 0)},{cmd_ticks.get(2, 0)},{cmd_ticks.get(3, 0)},{cmd_ticks.get(4, 0)},{cmd_ticks.get(5, 0)},{cmd_ticks.get(6, 0)},"
                    f"{actual_ticks.get(1, 0)},{actual_ticks.get(2, 0)},{actual_ticks.get(3, 0)},{actual_ticks.get(4, 0)},{actual_ticks.get(5, 0)},{actual_ticks.get(6, 0)}\n"
                )
                csv_file.flush()

                survey_data.append({
                    "row": row,
                    "col": col,
                    "target_x": target_xw,
                    "target_y": target_yw,
                    "act_x": act_xw,
                    "act_y": act_yw,
                    "err_r": err_r,
                    "is_reachable": is_reachable,
                    "status": status,
                })

        # Return safely to Home Pose
        print("\n[Traversal] Complete! Returning to Home Pose...")
        driver.send_multi_servo_move(home_ticks, time_ms=1600)
        record_frames_duration(2.0)

    finally:
        csv_file.close()
        video_writer.release()
        cap.release()
        driver.disconnect()
        print("[Hardware] Disconnected safely.")

    # 7. Compress Video via ffmpeg to H.264 MP4
    print(f"\n[Compression] Compressing raw video via ffmpeg to {output_video_path}...")
    cmd = [
        "ffmpeg", "-y",
        "-i", raw_avi_path,
        "-c:v", "libx264",
        "-preset", "fast",
        "-crf", "26",
        "-pix_fmt", "yuv420p",
        output_video_path,
    ]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        if os.path.exists(output_video_path):
            size_mb = os.path.getsize(output_video_path) / (1024 * 1024)
            print(f"[Compression] Success! Saved compressed video: {output_video_path} ({size_mb:.2f} MB)")
            if os.path.exists(raw_avi_path):
                os.remove(raw_avi_path)
    except Exception as e:
        print(f"[Compression Warning] ffmpeg encoding failed: {e}. Keeping raw AVI.")

    # 8. Generate 2D Kinematic Analysis Plot
    generate_analysis_plot(output_csv_path, "artifacts/grid_kinematics_analysis.png")
    brain_dir = "/home/shubham/.gemini/antigravity/brain/482e3c1f-9d32-47a6-9063-f90f94dfde00"
    if os.path.exists(brain_dir):
        import shutil
        if os.path.exists("artifacts/grid_kinematics_analysis.png"):
            shutil.copy("artifacts/grid_kinematics_analysis.png", os.path.join(brain_dir, "grid_kinematics_analysis.png"))
        if os.path.exists(output_video_path):
            shutil.copy(output_video_path, os.path.join(brain_dir, "arena_grid_traversal.mp4"))
    return True


def generate_analysis_plot(csv_path: str, output_img_path: str):
    """Parses telemetry CSV and generates visual 2D arena reachability & accuracy chart using OpenCV."""
    if not os.path.exists(csv_path):
        return

    data = []
    with open(csv_path, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            parts = line.strip().split(",")
            if len(parts) >= 20:
                data.append({
                    "row": int(parts[1]),
                    "col": int(parts[2]),
                    "tx": float(parts[3]),
                    "ty": float(parts[4]),
                    "ax": float(parts[15]),
                    "ay": float(parts[16]),
                    "err_r": float(parts[20]),
                    "reachable": parts[13] == "True",
                })

    if not data:
        return

    # Canvas dimensions
    img_w, img_h = 900, 950
    canvas = np.full((img_h, img_w, 3), 255, dtype=np.uint8)

    # Plot coordinate mapping: X in [-20, 370], Y in [-20, 420]
    margin_x, margin_y = 80, 80
    plot_w = img_w - 2 * margin_x
    plot_h = img_h - margin_y - 120
    x_min, x_max = -20.0, 370.0
    y_min, y_max = -20.0, 420.0

    def to_canvas(x: float, y: float) -> Tuple[int, int]:
        px = int(margin_x + (x - x_min) / (x_max - x_min) * plot_w)
        py = int(img_h - 100 - (y - y_min) / (y_max - y_min) * plot_h)
        return px, py

    # Draw grid lines (every 50mm)
    for x in range(0, 351, 50):
        p1 = to_canvas(x, 0)
        p2 = to_canvas(x, 400)
        cv2.line(canvas, p1, p2, (230, 230, 230), 1)
        cv2.putText(canvas, str(x), (p1[0] - 12, p1[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

    for y in range(0, 401, 50):
        p1 = to_canvas(0, y)
        p2 = to_canvas(350, y)
        cv2.line(canvas, p1, p2, (230, 230, 230), 1)
        cv2.putText(canvas, str(y), (p1[0] - 35, p1[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

    # Axis labels
    cv2.putText(canvas, "Arena X (mm, West -> East)", (img_w // 2 - 100, img_h - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1)
    cv2.putText(canvas, "Arena Y (mm)", (15, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1)

    # Arena Perimeter box (X: 25..325, Y: 25..275)
    tl = to_canvas(25.0, 275.0)
    br = to_canvas(325.0, 25.0)
    cv2.rectangle(canvas, (tl[0], tl[1]), (br[0], br[1]), (80, 80, 80), 2)
    cv2.putText(canvas, "Arena Working Grid (Rows 6-11)", (tl[0] + 10, tl[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)

    # xArm Base at (175, 375)
    b_pt = to_canvas(175.0, 375.0)
    cv2.circle(canvas, b_pt, 12, (0, 0, 0), -1)
    cv2.putText(canvas, "xArm Base (175, 375)", (b_pt[0] - 70, b_pt[1] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)

    # ArUco Markers
    markers = {
        "Marker 2 (6,0)": (25.0, 275.0),
        "Marker 0 (6,6)": (325.0, 275.0),
        "Marker 3 (11,0)": (25.0, 25.0),
        "Marker 1 (11,6)": (325.0, 25.0),
    }
    for mname, (mx, my) in markers.items():
        m_pt = to_canvas(mx, my)
        cv2.circle(canvas, m_pt, 9, (200, 0, 200), -1)
        offset_y = -12 if my > 100 else 22
        cv2.putText(canvas, mname, (m_pt[0] - 40, m_pt[1] + offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 0, 150), 1)

    # Plot points
    reachable_cnt = 0
    total_cnt = len(data)
    errors = []
    for d in data:
        t_pt = to_canvas(d["tx"], d["ty"])
        a_pt = to_canvas(d["ax"], d["ay"])
        if d["reachable"]:
            reachable_cnt += 1
            errors.append(d["err_r"])
            # Target (Green circle)
            cv2.circle(canvas, t_pt, 6, (0, 180, 0), -1)
            # Reached (Blue circle)
            cv2.circle(canvas, a_pt, 4, (200, 50, 0), -1)
            # Error arrow
            cv2.line(canvas, t_pt, a_pt, (150, 50, 0), 1)
        else:
            # Unreachable Target (Red square)
            cv2.rectangle(canvas, (t_pt[0] - 6, t_pt[1] - 6), (t_pt[0] + 6, t_pt[1] + 6), (0, 0, 220), -1)
            # Clamped Reached (Orange diamond)
            cv2.circle(canvas, a_pt, 4, (0, 140, 255), -1)
            cv2.line(canvas, t_pt, a_pt, (0, 140, 255), 1)

    mean_err = float(np.mean(errors)) if errors else 0.0

    # Title & Stats Banner
    cv2.putText(canvas, "xArm 1S Kinematics Arena Survey: 42-Point Grid Traversal", (margin_x, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
    stats_str = f"Reachable: {reachable_cnt}/{total_cnt} ({reachable_cnt/total_cnt*100:.1f}%) | Mean Radial Tracking Error: {mean_err:.2f} mm"
    cv2.putText(canvas, stats_str, (margin_x, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 60), 1)

    # Legend at bottom
    leg_y = img_h - 18
    # Green reachable
    cv2.circle(canvas, (margin_x + 10, leg_y - 4), 5, (0, 180, 0), -1)
    cv2.putText(canvas, "Reachable Target", (margin_x + 22, leg_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    # Blue actual
    cv2.circle(canvas, (margin_x + 160, leg_y - 4), 4, (200, 50, 0), -1)
    cv2.putText(canvas, "Actual Reached (FK)", (margin_x + 172, leg_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    # Red unreachable
    cv2.rectangle(canvas, (margin_x + 330, leg_y - 8), (margin_x + 338, leg_y), (0, 0, 220), -1)
    cv2.putText(canvas, "Out of Envelope (Target)", (margin_x + 344, leg_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
    # Orange clamped
    cv2.circle(canvas, (margin_x + 520, leg_y - 4), 4, (0, 140, 255), -1)
    cv2.putText(canvas, "Clamped Boundary (FK)", (margin_x + 532, leg_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    os.makedirs(os.path.dirname(output_img_path), exist_ok=True)
    cv2.imwrite(output_img_path, canvas)
    print(f"[Analysis] Saved 2D kinematic analysis plot: {output_img_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="xArm Arena Grid Kinematic Survey")
    parser.add_argument("--device", default="/dev/video4", help="Camera device path (default: /dev/video4)")
    parser.add_argument("--port", default="/dev/hidraw3", help="Arm USB device path (default: /dev/hidraw3)")
    parser.add_argument("--z", type=float, default=35.0, help="Hover altitude above table in mm (default: 35.0)")
    parser.add_argument("--hold", type=float, default=0.5, help="Hold dwell time at each cell in seconds (default: 0.5)")
    parser.add_argument("--dry-run", action="store_true", help="Simulate arm motion without physical hardware")
    args = parser.parse_args()

    run_survey(
        device_path=args.device,
        arm_port=args.port,
        safe_z_mm=args.z,
        hold_time_sec=args.hold,
        dry_run=args.dry_run,
    )
