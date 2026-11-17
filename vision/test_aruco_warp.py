#!/usr/bin/env python3
"""
ArUco Marker Planar Homography & Rectification Verification Script.

Features:
  - Detects 4 arena ArUco markers (DICT_4X4_50, IDs: 0, 1, 2, 3) at Z = 0.0 mm.
  - Computes perspective transform matrix H to warp the arena to a metric grid (2.0 px/mm).
  - Press 'l' to lock/freeze homography matrix (guards against vibrations and arm occlusions).
  - Press 'g' to toggle metric millimeter grid overlay for physical ruler validation.
  - Press 's' to save locked homography matrix directly to config/camera_calibration.yaml.
  - Supports --synthetic mode to run offline automated tests without a physical webcam.
"""

import argparse
import os
import sys
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(script_dir, ".."))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

import cv2
import numpy as np
from vision.arena_vision_pipeline import ArenaVisionPipeline
import yaml


def generate_synthetic_arena_frame(width=1920, height=1080) -> np.ndarray:
    """Creates a synthetic camera frame with 4 ArUco markers and colored cubes for offline testing."""
    frame = np.full((height, width, 3), 40, dtype=np.uint8)

    # Arena bounding box in full frame (approximate camera view)
    pts_src = np.array([
        [450, 200],   # Marker 0: Top-Left
        [1470, 200],  # Marker 1: Top-Right
        [1540, 880],  # Marker 2: Bottom-Right
        [380, 880],   # Marker 3: Bottom-Left
    ], dtype=np.float32)

    # Draw arena background
    arena_poly = pts_src.astype(np.int32)
    cv2.fillConvexPoly(frame, arena_poly, (80, 80, 80))

    # Generate ArUco marker images and blend onto frame
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size = 90

    for i, marker_id in enumerate([0, 1, 2, 3]):
        # Generate marker image
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        else:
            marker_img = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

        cx, cy = int(pts_src[i][0]), int(pts_src[i][1])
        x1, y1 = cx - marker_size // 2, cy - marker_size // 2
        x2, y2 = x1 + marker_size, y1 + marker_size
        
        # Add white border
        cv2.rectangle(frame, (x1 - 6, y1 - 6), (x2 + 6, y2 + 6), (255, 255, 255), -1)
        frame[y1:y2, x1:x2] = marker_bgr

    # Draw synthetic colored cubes inside arena
    # Compute rough homography to place cubes at known physical coordinates
    pipeline = ArenaVisionPipeline()
    h_synth = cv2.getPerspectiveTransform(pts_src, pipeline.dst_pts)
    h_inv = np.linalg.inv(h_synth)

    # Sample cubes inside 260x140 mm arena (X in [70, 330], Y in [40, 180])
    test_cubes = [
        {"x": 150.0, "y": 140.0, "color": (0, 0, 255)},    # Red (r ~ 148 mm)
        {"x": 280.0, "y": 130.0, "color": (255, 0, 0)},    # Blue (r ~ 170 mm)
        {"x": 220.0, "y": 80.0,  "color": (0, 255, 255)},  # Yellow (r ~ 201 mm)
        {"x": 130.0, "y": 70.0,  "color": (0, 255, 0)},    # Green (r ~ 221 mm)
    ]

    cube_half_w = 25.0  # 25 px half-width = 12.5 mm in 2.0 px/mm -> 50x50 px = 25x25 mm cube
    for cube in test_cubes:
        u, v = pipeline.world_to_pixel(cube["x"], cube["y"])
        corners_metric = np.array([
            [u - cube_half_w, v - cube_half_w],
            [u + cube_half_w, v - cube_half_w],
            [u + cube_half_w, v + cube_half_w],
            [u - cube_half_w, v + cube_half_w],
        ], dtype=np.float32).reshape(-1, 1, 2)
        corners_cam = cv2.perspectiveTransform(corners_metric, h_inv).astype(np.int32)
        cv2.fillConvexPoly(frame, corners_cam, cube["color"])
        cv2.polylines(frame, [corners_cam], True, (0, 0, 0), 2)

    return frame


def save_homography_to_yaml(yaml_path: str, h_matrix: np.ndarray, scale: float = 2.0):
    """Saves calculated planar homography matrix into config/camera_calibration.yaml."""
    if not os.path.exists(yaml_path):
        print(f"[Error] YAML file {yaml_path} does not exist.")
        return

    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        data["planar_homography"] = {
            "scale_px_per_mm": float(scale),
            "matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(round(v, 6)) for v in h_matrix.flatten()],
            },
        }

        temp_path = yaml_path + ".tmp"
        with open(temp_path, "w") as f:
            yaml.dump(data, f, sort_keys=False, default_flow_style=False)
        os.replace(temp_path, yaml_path)
        print(f"[Success] Planar homography matrix saved to {yaml_path}!")
    except Exception as e:
        print(f"[Error] Failed to save homography: {e}")


def main():
    parser = argparse.ArgumentParser(description="Verify 4-point ArUco planar homography & metric rectification.")
    parser.add_argument("--device", default="/dev/video0", help="Camera device path or index (default: /dev/video0)")
    parser.add_argument("--config", default="config/camera_calibration.yaml", help="Path to camera_calibration.yaml")
    parser.add_argument("--scale", type=float, default=2.0, help="Metric scale px/mm (default: 2.0)")
    parser.add_argument("--synthetic", action="store_true", help="Run offline verification on synthetic arena frame")
    args = parser.parse_args()

    pipeline = ArenaVisionPipeline(config_dir=os.path.dirname(args.config), scale_px_per_mm=args.scale)

    print("========================================================================")
    print("  ARUCO PLANAR HOMOGRAPHY & METRIC WARP VERIFICATION")
    print(f"  Target Grid: 780x420 px @ {args.scale} px/mm (390x210 mm physical arena)")
    print("  CONTROLS:")
    print("    [l] : Lock / Freeze Homography Matrix (guards against vibrations)")
    print("    [g] : Toggle Metric Millimeter Grid (50 mm major grid)")
    print(f"    [s] : Save current homography to {args.config}")
    print("    [q] : Quit verification")
    print("========================================================================")

    draw_grid = True

    if args.synthetic:
        print("[Synthetic] Testing ArUco detection on synthetic arena frame...")
        synthetic_frame = generate_synthetic_arena_frame()
        success = pipeline.calibrate_homography(synthetic_frame)
        if not success:
            print("[Error] Failed to detect all 4 ArUco markers on synthetic frame.")
            sys.exit(1)

        print("[Synthetic] All 4 ArUco markers (0, 1, 2, 3) detected successfully!")
        warped, annotated, cubes = pipeline.process_frame(synthetic_frame, draw_grid=draw_grid)
        print(f"[Synthetic] Rectified frame size: {warped.shape[1]}x{warped.shape[0]} px")
        print(f"[Synthetic] Detected {len(cubes)} cubes:")
        for c in cubes:
            print(f"  - {c.color.upper()}: World=({c.world_x:.1f}, {c.world_y:.1f}) mm | "
                  f"Target Arm={c.assigned_arm} | r={c.target_r_mm:.1f}mm | th={c.target_theta_deg:+.1f}°")

        # Save synthetic homography
        save_homography_to_yaml(args.config, pipeline.H_matrix, scale=args.scale)
        print("[Synthetic] Offline test PASSED successfully.")
        return

    # Real camera loop
    dev_id = int(args.device) if args.device.isdigit() else args.device
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2 if isinstance(dev_id, str) else cv2.CAP_ANY)
    if not cap.isOpened():
        print(f"[Error] Could not open camera {args.device}. Use --synthetic to test offline.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        raw_display = frame.copy()
        corners, ids = pipeline.detect_aruco_markers(raw_display)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(raw_display, corners, ids)

        # Calibrate / update homography if not locked
        calib_ok = pipeline.calibrate_homography(frame)
        warped, annotated, cubes = pipeline.process_frame(frame, draw_grid=draw_grid)

        # Draw lock status indicator
        lock_status = "HOMOGRAPHY LOCKED" if pipeline.H_locked else "TRACKING LIVE"
        lock_color = (0, 255, 0) if pipeline.H_locked else (0, 165, 255)
        cv2.putText(raw_display, f"Status: {lock_status} (Press 'l' to toggle)", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, lock_color, 2, cv2.LINE_AA)

        # Display raw detection
        cv2.imshow("Overhead Camera (ArUco Detection)", cv2.resize(raw_display, (960, 540)))

        # Display rectified metric arena
        if annotated is not None:
            cv2.imshow("Rectified Metric Arena (2.0 px/mm)", annotated)

        key = cv2.waitKey(20) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('l'):
            pipeline.lock_homography(not pipeline.H_locked)
            print(f"[Homography] Lock state changed: {pipeline.H_locked}")
        elif key == ord('g'):
            draw_grid = not draw_grid
            print(f"[Display] Grid display: {draw_grid}")
        elif key == ord('s'):
            if pipeline.H_matrix is not None:
                save_homography_to_yaml(args.config, pipeline.H_matrix, scale=args.scale)
            else:
                print("[Warning] No valid homography matrix to save. Detect 4 markers first.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
