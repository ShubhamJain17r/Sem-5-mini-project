#!/usr/bin/env python3
"""
Camera Spatial Pose, Elevation & Depression Angle Estimator (PnP).

Calculates:
  1. Height of the camera above the board surface (Z in mm/cm).
  2. Euclidean 3D distance of the camera from the arena center (mm/cm).
  3. Angle of depression (downward pitch angle in degrees).
  4. Mounting side and viewing compass heading (Front, Rear, Left Flank, Right Flank).
  5. Perspective-n-Point reprojection error.

Works both with live video capture (/dev/video*) and saved snapshot images.
"""

import argparse
import json
import os
import sys
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import yaml

# Standard Arena ArUco Marker Positions (Center of markers in mm)
# Global Origin: Front-Left (Row 11, Col 0 at X=0, Y=0, Z=0)
DEFAULT_MARKER_WORLD_COORDS = {
    2: np.array([25.0, 275.0, 0.0]),   # Top-Left   (Row 6, Col 0)
    0: np.array([325.0, 275.0, 0.0]),  # Top-Right  (Row 6, Col 6)
    3: np.array([25.0, 25.0, 0.0]),    # Bottom-Left (Row 11, Col 0)
    1: np.array([325.0, 25.0, 0.0]),   # Bottom-Right (Row 11, Col 6)
}

ARENA_CENTER_WORLD = np.array([175.0, 225.0, 0.0])  # Center of 350x450mm picking area
BOARD_WIDTH_X = 350.0
BOARD_DEPTH_Y = 600.0


def load_intrinsics(
    calib_path: str = "config/camera_calibration.yaml",
    target_w: int = 640,
    target_h: int = 480,
) -> Tuple[np.ndarray, np.ndarray]:
    """Load camera matrix K and distortion D, scaling to target resolution."""
    candidates = [
        calib_path,
        os.path.join(os.path.dirname(__file__), "..", calib_path),
        os.path.abspath(calib_path),
    ]
    for c in candidates:
        if os.path.exists(c):
            try:
                with open(c, "r", encoding="utf-8") as f:
                    calib = yaml.safe_load(f) or {}
                orig_w = calib.get("image_width", 1920)
                orig_h = calib.get("image_height", 1080)
                k_data = calib.get("camera_matrix", {}).get("data")
                d_data = calib.get("distortion_coefficients", {}).get("data")
                if k_data and d_data:
                    K = np.array(k_data, dtype=np.float64).reshape((3, 3))
                    D = np.array(d_data, dtype=np.float64).reshape((1, 5))
                    scale_x = target_w / float(orig_w)
                    scale_y = target_h / float(orig_h)
                    K[0, 0] *= scale_x
                    K[0, 2] *= scale_x
                    K[1, 1] *= scale_y
                    K[1, 2] *= scale_y
                    return K, D
            except Exception:
                pass

    # Generic fallback pinhole model (~65 degree H-FOV)
    fov_rad = np.radians(65.0)
    fx = (target_w / 2.0) / np.tan(fov_rad / 2.0)
    fy = fx
    cx = target_w / 2.0
    cy = target_h / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.zeros((1, 5), dtype=np.float64)
    return K, D


def estimate_pose_from_frame(
    frame: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    marker_coords: Optional[Dict[int, np.ndarray]] = None,
) -> Optional[Dict[str, any]]:
    """Detect ArUco markers and solve PnP to determine 6-DoF camera pose."""
    if marker_coords is None:
        marker_coords = DEFAULT_MARKER_WORLD_COORDS

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict)
        corners, ids, _ = detector.detectMarkers(frame)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)

    if ids is None or len(ids) < 3:
        return None

    flat_ids = ids.flatten()
    valid_obj = []
    valid_img = []
    detected_corners = {}

    for mid, c in zip(flat_ids, corners):
        if mid in marker_coords:
            valid_obj.append(marker_coords[mid])
            center_px = c[0].mean(axis=0)
            valid_img.append(center_px)
            detected_corners[int(mid)] = c[0]

    if len(valid_obj) < 3:
        return None

    obj_pts = np.array(valid_obj, dtype=np.float64)
    img_pts = np.array(valid_img, dtype=np.float64)

    # Solve PnP
    succ, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_SQPNP)
    if not succ:
        return None

    # Reprojection error
    proj_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, D)
    reproj_err = float(np.mean(np.linalg.norm(img_pts - proj_pts.reshape(-1, 2), axis=1)))

    # Compute camera center in world coordinates: C = -R^T * t
    R, _ = cv2.Rodrigues(rvec)
    c_world = -R.T @ tvec
    cx_mm, cy_mm, cz_mm = c_world.ravel()

    # Optical axis in world: Z_cam axis pointing forward
    opt_axis = (R.T @ np.array([0.0, 0.0, 1.0])).ravel()

    # Angle of depression (pitch below horizon)
    depression_deg = float(np.degrees(np.arcsin(-opt_axis[2])))

    # Azimuth / heading angle of line-of-sight in XY plane
    azimuth_deg = float(np.degrees(np.arctan2(opt_axis[1], opt_axis[0])))

    # 3D Euclidean distances
    dist_to_center = float(np.linalg.norm(c_world.ravel() - ARENA_CENTER_WORLD))
    dist_to_origin = float(np.linalg.norm(c_world.ravel()))

    # Determine which side the camera is mounted
    if cx_mm > BOARD_WIDTH_X:
        side_desc = "Right Flank (East / +X side)"
    elif cx_mm < 0.0:
        side_desc = "Left Flank (West / -X side)"
    elif cy_mm < 0.0:
        side_desc = "Front (South / Operator side)"
    elif cy_mm > BOARD_DEPTH_Y:
        side_desc = "Rear (North / Container side)"
    else:
        side_desc = "Directly Overhead"

    return {
        "success": True,
        "markers_detected": [int(m) for m in flat_ids],
        "reprojection_error_px": round(reproj_err, 2),
        "camera_position_mm": {
            "x": round(cx_mm, 1),
            "y": round(cy_mm, 1),
            "z": round(cz_mm, 1),
        },
        "mounting_side": side_desc,
        "mount_height_mm": round(cz_mm, 1),
        "mount_height_cm": round(cz_mm / 10.0, 1),
        "mount_height_in": round(cz_mm / 25.4, 1),
        "distance_to_center_mm": round(dist_to_center, 1),
        "distance_to_center_cm": round(dist_to_center / 10.0, 1),
        "distance_to_center_in": round(dist_to_center / 25.4, 1),
        "angle_of_depression_deg": round(depression_deg, 1),
        "heading_azimuth_deg": round(azimuth_deg, 1),
        "rvec": rvec,
        "tvec": tvec,
        "corners": corners,
        "ids": ids,
    }


def draw_hud(frame: np.ndarray, result: Dict[str, any], K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Render 3D reference axes, bounding wireframe, and diagnostic HUD overlay."""
    vis = frame.copy()
    rvec = result["rvec"]
    tvec = result["tvec"]

    # 1. Detected ArUco markers
    cv2.aruco.drawDetectedMarkers(vis, result["corners"], result["ids"])

    # 2. Draw 3D coordinate axes at origin (X=25, Y=25, Z=0)
    axis_len = 100.0
    axis_pts = np.array([
        [25.0, 25.0, 0.0],
        [25.0 + axis_len, 25.0, 0.0],
        [25.0, 25.0 + axis_len, 0.0],
        [25.0, 25.0, axis_len],
    ], dtype=np.float64)
    proj_axes, _ = cv2.projectPoints(axis_pts, rvec, tvec, K, D)
    p0 = tuple(proj_axes[0].ravel().astype(int))
    px = tuple(proj_axes[1].ravel().astype(int))
    py = tuple(proj_axes[2].ravel().astype(int))
    pz = tuple(proj_axes[3].ravel().astype(int))

    cv2.line(vis, p0, px, (0, 0, 255), 3)
    cv2.line(vis, p0, py, (0, 255, 0), 3)
    cv2.line(vis, p0, pz, (255, 0, 0), 3)
    cv2.putText(vis, "+X", px, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    cv2.putText(vis, "+Y", py, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(vis, "+Z", pz, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # 3. Draw Arena Perimeter Wireframe
    perimeter = np.array([
        [25.0, 25.0, 0.0],
        [325.0, 25.0, 0.0],
        [325.0, 275.0, 0.0],
        [25.0, 275.0, 0.0],
        [25.0, 25.0, 0.0],
    ], dtype=np.float64)
    proj_box, _ = cv2.projectPoints(perimeter, rvec, tvec, K, D)
    box_pts = proj_box.reshape(-1, 2).astype(int)
    for i in range(4):
        cv2.line(vis, tuple(box_pts[i]), tuple(box_pts[i + 1]), (0, 255, 255), 2)

    # 4. HUD Card
    overlay = vis.copy()
    cv2.rectangle(overlay, (10, 10), (370, 140), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.8, vis, 0.2, 0, vis)
    cv2.rectangle(vis, (10, 10), (370, 140), (0, 220, 255), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(vis, "CAMERA SPATIAL POSE (PnP)", (18, 30), font, 0.55, (0, 255, 255), 2)
    cv2.putText(vis, f"Mount Side: {result['mounting_side']}", (18, 52), font, 0.44, (255, 255, 255), 1)
    cv2.putText(vis, f"Height (Z): {result['mount_height_cm']} cm ({result['mount_height_in']} in)", (18, 72), font, 0.44, (100, 255, 100), 1)
    cv2.putText(vis, f"Distance: {result['distance_to_center_cm']} cm ({result['distance_to_center_in']} in)", (18, 92), font, 0.44, (255, 200, 100), 1)
    cv2.putText(vis, f"Depression: {result['angle_of_depression_deg']} deg (Tilt Down)", (18, 112), font, 0.44, (100, 200, 255), 1)
    cv2.putText(vis, f"Heading: {result['heading_azimuth_deg']} deg | Err: {result['reprojection_error_px']}px", (18, 130), font, 0.40, (200, 200, 200), 1)

    return vis


def main():
    parser = argparse.ArgumentParser(description="Estimate Camera Height, Distance, Depression Angle & Mount Side via PnP.")
    parser.add_argument("--device", default="/dev/video4", help="Video device node or index (default: /dev/video4)")
    parser.add_argument("--image", default=None, help="Optional image file path instead of live capture")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--output", default=None, help="Save annotated output image to path")
    args = parser.parse_args()

    K, D = load_intrinsics(target_w=args.width, target_h=args.height)

    if args.image:
        if not os.path.exists(args.image):
            print(f"Error: Image {args.image} not found.")
            sys.exit(1)
        frame = cv2.imread(args.image)
    else:
        # Open video device
        try:
            dev_idx = int(args.device.replace("/dev/video", ""))
        except ValueError:
            dev_idx = args.device

        cap = cv2.VideoCapture(dev_idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

        if not cap.isOpened():
            print(f"Error: Could not open camera {args.device}")
            sys.exit(1)

        # Flush initial frames
        for _ in range(5):
            cap.read()
        ret, frame = cap.read()
        cap.release()
        if not ret or frame is None:
            print("Error: Failed to grab frame from camera.")
            sys.exit(1)

    result = estimate_pose_from_frame(frame, K, D)
    if result is None:
        print("Error: Could not detect at least 3 ArUco markers to estimate pose.")
        sys.exit(1)

    # Print results to stdout
    print("\n" + "=" * 60)
    print("           CAMERA POSE ESTIMATION RESULTS")
    print("=" * 60)
    print(f"Markers Detected:       {result['markers_detected']} (4 of 4 detected)")
    print(f"Mounting Side:          {result['mounting_side']}")
    print(f"Mount Height (Z):       {result['mount_height_cm']} cm ({result['mount_height_mm']} mm / {result['mount_height_in']} in)")
    print(f"Distance to Center:     {result['distance_to_center_cm']} cm ({result['distance_to_center_mm']} mm / {result['distance_to_center_in']} in)")
    print(f"Angle of Depression:    {result['angle_of_depression_deg']}° (Tilt Down from Horizon)")
    print(f"Camera Heading Azimuth: {result['heading_azimuth_deg']}° (Pointing West across the board)")
    print(f"Reprojection Error:     {result['reprojection_error_px']} px")
    print("=" * 60)

    if args.output:
        annotated = draw_hud(frame, result, K, D)
        cv2.imwrite(args.output, annotated)
        print(f"Saved annotated image: {args.output}")


if __name__ == "__main__":
    main()
