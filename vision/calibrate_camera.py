#!/usr/bin/env python3
"""
Camera Intrinsic Calibration Tool for Overhead Boom Webcam.

Features:
  - Detects checkerboard corners (default 9x6 inner corners, 25mm square size).
  - Collects calibration samples on keypress (SPACE).
  - Computes camera intrinsic matrix K and distortion coefficients D.
  - Updates config/camera_calibration.yaml with calibration date and reprojection error.
  - Supports --synthetic mode to run offline automated tests without a physical camera.
"""

import argparse
from datetime import datetime
import os
import sys
import time

import cv2
import numpy as np
import yaml


def run_synthetic_calibration(output_yaml: str):
    """Generates synthetic calibration parameters and writes them to YAML."""
    print("[Calibration] Running synthetic calibration mode...")
    k_synthetic = [
        1425.0,    0.0, 960.0,
           0.0, 1425.0, 540.0,
           0.0,    0.0,   1.0
    ]
    d_synthetic = [-0.045, 0.021, 0.0008, -0.0005, 0.0]

    update_calibration_yaml(output_yaml, k_synthetic, d_synthetic, reprojection_error=0.18, width=1920, height=1080)
    print(f"[Calibration] Synthetic calibration saved to {output_yaml} successfully.")


def update_calibration_yaml(
    yaml_path: str,
    camera_matrix: list,
    distortion_coeffs: list,
    reprojection_error: float,
    width: int = 1920,
    height: int = 1080,
):
    """Atomically updates config/camera_calibration.yaml with new calibration data."""
    existing_data = {}
    if os.path.exists(yaml_path):
        try:
            with open(yaml_path, "r") as f:
                existing_data = yaml.safe_load(f) or {}
        except Exception as e:
            print(f"[Calibration] Warning reading existing YAML: {e}")

    existing_data["version"] = "1.0"
    existing_data["date_calibrated"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    existing_data["camera_name"] = "overhead_camera"
    existing_data["image_width"] = width
    existing_data["image_height"] = height
    existing_data["reprojection_error_px"] = float(round(reprojection_error, 4))

    existing_data["camera_matrix"] = {
        "rows": 3,
        "cols": 3,
        "data": [float(round(v, 4)) for v in camera_matrix],
    }
    existing_data["distortion_coefficients"] = {
        "rows": 1,
        "cols": len(distortion_coeffs),
        "data": [float(round(v, 6)) for v in distortion_coeffs],
    }

    # Atomic write via temp file
    temp_path = yaml_path + ".tmp"
    with open(temp_path, "w") as f:
        yaml.dump(existing_data, f, sort_keys=False, default_flow_style=False)
    os.replace(temp_path, yaml_path)


def main():
    parser = argparse.ArgumentParser(description="Calibrate overhead camera intrinsic lens parameters.")
    parser.add_argument("--device", default="/dev/video0", help="V4L2 camera device path or index (default: /dev/video0)")
    parser.add_argument("--config", default="config/camera_calibration.yaml", help="Path to camera_calibration.yaml")
    parser.add_argument("--rows", type=int, default=6, help="Checkerboard inner row corners (default: 6)")
    parser.add_argument("--cols", type=int, default=9, help="Checkerboard inner column corners (default: 9)")
    parser.add_argument("--square-size", type=float, default=25.0, help="Square side length in mm (default: 25.0)")
    parser.add_argument("--min-samples", type=int, default=15, help="Minimum captured samples before calibration (default: 15)")
    parser.add_argument("--synthetic", action="store_true", help="Run offline synthetic calibration without webcam")
    args = parser.parse_args()

    if args.synthetic:
        run_synthetic_calibration(args.config)
        return

    # Prepare 3D object points in physical millimeter units
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # Open camera
    dev_id = int(args.device) if args.device.isdigit() else args.device
    cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2 if isinstance(dev_id, str) else cv2.CAP_ANY)
    if not cap.isOpened():
        print(f"[Error] Could not open camera {args.device}. Use --synthetic to test offline.")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("========================================================================")
    print("  OVERHEAD CAMERA INTRINSIC CALIBRATION")
    print(f"  Target Grid: {args.cols}x{args.rows} inner corners, {args.square_size}mm squares")
    print("  CONTROLS:")
    print("    [SPACE] : Capture current frame if checkerboard is detected")
    print(f"    [c]     : Compute calibration and save to {args.config}")
    print("    [q]     : Quit without saving")
    print("========================================================================")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    captured_count = 0
    img_size = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[Warning] Failed to grab frame.")
            time.sleep(0.1)
            continue

        if img_size is None:
            img_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (args.cols, args.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (args.cols, args.rows), corners, found)
            status_text = "Checkerboard DETECTED! Press SPACE to capture"
            status_color = (0, 255, 0)
        else:
            status_text = "Searching for checkerboard..."
            status_color = (0, 165, 255)

        cv2.putText(display, f"Samples: {captured_count}/{args.min_samples}", (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(display, status_text, (30, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2, cv2.LINE_AA)

        cv2.imshow("Camera Calibration", display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord(' '):
            if found:
                refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(refined_corners)
                captured_count += 1
                print(f"[Captured] Sample #{captured_count} recorded.")
            else:
                print("[Warning] Checkerboard not detected in current frame. Adjust position.")

        elif key == ord('c'):
            if captured_count < args.min_samples:
                print(f"[Warning] Only {captured_count} samples collected. Recommended minimum is {args.min_samples}.")
                ans = input("Proceed with calibration anyway? (y/n): ")
                if ans.lower() != 'y':
                    continue

            print("[Calibration] Computing camera matrix and distortion coefficients...")
            ret_val, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )

            # Compute reprojection error
            total_error = 0.0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                total_error += error
            mean_error = total_error / len(objpoints)

            print(f"[Results] Reprojection Error: {mean_error:.4f} pixels")
            print(f"[Results] Camera Matrix K:\n{mtx}")
            print(f"[Results] Distortion D:\n{dist.ravel()}")

            update_calibration_yaml(
                args.config,
                mtx.flatten().tolist(),
                dist.flatten().tolist(),
                reprojection_error=mean_error,
                width=img_size[0],
                height=img_size[1]
            )
            print(f"[Success] Updated {args.config} successfully!")
            break

        elif key == ord('q'):
            print("[Quit] Exiting without saving.")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
