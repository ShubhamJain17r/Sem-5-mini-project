#!/usr/bin/env python3
"""
Overhead Camera Centering & Tilt Alignment Program.

Helps physical setup of the overhead camera stand:
  - Detects the 4 corner ArUco calibration markers (DICT_4X4_50, IDs 0 to 3).
  - Computes the camera's optical center reticle (cx, cy).
  - Computes the physical arena center in image coordinates via projective diagonal intersection.
  - Measures Centering Offset (ΔX, ΔY in pixels and millimeters).
  - Measures Keystoning / Perspective Tilt (Pitch ratio, Yaw ratio, and Roll rotation angle).
  - Displays real-time OpenCV HUD with directional mounting prompts (PAN, TILT, ROTATE).
  - Provides visual status transitions: RED (Misaligned) -> YELLOW (Close) -> GREEN (ALIGNED).
  - Saves alignment calibration parameters to config/camera_alignment.json upon pressing 's'.
  - Supports --synthetic mode for automated headless verification.
"""

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# Marker physical coordinates (mm)
MARKER_PHYSICAL_COORDS = {
    0: (70.0, 180.0),   # Rear-Left / Top-Left
    1: (330.0, 180.0),  # Rear-Right / Top-Right
    2: (70.0, 40.0),    # Front-Left / Bottom-Left
    3: (330.0, 40.0),   # Front-Right / Bottom-Right
}
ARENA_CENTER_PHYSICAL = (200.0, 110.0)


def compute_diagonal_intersection(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    p3: Tuple[float, float],
) -> Tuple[float, float]:
    """
    Compute intersection point of diagonals (p0 <-> p3) and (p1 <-> p2).
    In projective geometry, diagonal intersection is invariant to projective perspective distortion.
    """
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # Line 1: (x0, y0) to (x3, y3)
    # Line 2: (x1, y1) to (x2, y2)
    denom = (x0 - x3) * (y1 - y2) - (y0 - y3) * (x1 - x2)
    if abs(denom) < 1e-6:
        # Fallback to mean center if lines are degenerate
        return ((x0 + x1 + x2 + x3) / 4.0, (y0 + y1 + y2 + y3) / 4.0)

    t1 = x0 * y3 - y0 * x3
    t2 = x1 * y2 - y1 * x2
    x = (t1 * (x1 - x2) - (x0 - x3) * t2) / denom
    y = (t1 * (y1 - y2) - (y0 - y3) * t2) / denom
    return (float(x), float(y))


def load_camera_intrinsics(calib_path: str = "config/camera_calibration.yaml") -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[int], Optional[int]]:
    """Load camera matrix and distortion coefficients if available."""
    candidates = [
        calib_path,
        os.path.join(os.getcwd(), calib_path),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", calib_path),
        "/workspace/" + calib_path,
    ]
    for c in candidates:
        if os.path.exists(c):
            try:
                with open(c, "r") as f:
                    data = yaml.safe_load(f) or {}
                k_data = data.get("camera_matrix", {}).get("data")
                d_data = data.get("distortion_coefficients", {}).get("data")
                cw = data.get("image_width")
                ch = data.get("image_height")
                if k_data and d_data:
                    k = np.array(k_data, dtype=np.float64).reshape((3, 3))
                    d = np.array(d_data, dtype=np.float64).reshape((1, 5))
                    return k, d, cw, ch
            except Exception:
                pass
    return None, None, None, None


class CameraAligner:
    """Detects arena ArUco markers and calculates alignment metrics."""

    def __init__(self, config_dir: str = "config", px_per_mm: float = 2.0):
        self.config_dir = config_dir
        self.px_per_mm = px_per_mm
        calib_file = os.path.join(config_dir, "camera_calibration.yaml")
        self.k_matrix, self.dist_coeffs, self.calib_w, self.calib_h = load_camera_intrinsics(calib_file)

        # Setup ArUco detector (OpenCV 4.7+ vs older)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        else:
            self.detector = None

    def detect_markers(self, frame: np.ndarray) -> Dict[int, Tuple[float, float]]:
        """Detect the 4 arena markers and return dict of {marker_id: (center_x, center_y)}."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=params)

        marker_centers = {}
        if ids is not None and len(ids) > 0:
            for i, mid in enumerate(ids.flatten()):
                if int(mid) in [0, 1, 2, 3]:
                    pts = corners[i][0]
                    cx = float(np.mean(pts[:, 0]))
                    cy = float(np.mean(pts[:, 1]))
                    marker_centers[int(mid)] = (cx, cy)
        return marker_centers

    def compute_metrics(
        self,
        frame_shape: Tuple[int, int],
        marker_centers: Dict[int, Tuple[float, float]],
    ) -> Dict[str, Any]:
        """Compute optical center, arena center, offset, tilt ratios, and roll angle."""
        h, w = frame_shape[:2]
        if self.k_matrix is not None and self.calib_w and self.calib_w > 0:
            scale_x = float(w) / float(self.calib_w)
            scale_y = float(h) / float(self.calib_h) if self.calib_h else scale_x
            cam_cx = float(self.k_matrix[0, 2]) * scale_x
            cam_cy = float(self.k_matrix[1, 2]) * scale_y
        else:
            cam_cx = float(w) / 2.0
            cam_cy = float(h) / 2.0

        all_present = all(i in marker_centers for i in [0, 1, 2, 3])
        if not all_present:
            missing = [i for i in [0, 1, 2, 3] if i not in marker_centers]
            return {
                "detected": False,
                "missing_markers": missing,
                "cam_center": (cam_cx, cam_cy),
                "frame_size": (w, h),
            }

        p0 = marker_centers[0]
        p1 = marker_centers[1]
        p2 = marker_centers[2]
        p3 = marker_centers[3]

        arena_center = compute_diagonal_intersection(p0, p1, p2, p3)
        arena_cx, arena_cy = arena_center

        # Pixel offsets (camera optical center - arena center)
        delta_x_px = cam_cx - arena_cx
        delta_y_px = cam_cy - arena_cy

        # Scale estimate from top edge (known physical length: 330.0 - 70.0 = 260.0 mm)
        d_top = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        d_bottom = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
        d_left = math.hypot(p2[0] - p0[0], p2[1] - p0[1])
        d_right = math.hypot(p3[0] - p1[0], p3[1] - p1[1])

        scale_px_per_mm = d_top / 260.0 if d_top > 1e-3 else self.px_per_mm
        delta_x_mm = delta_x_px / scale_px_per_mm
        delta_y_mm = delta_y_px / scale_px_per_mm

        # Keystoning ratios (nominal = 1.0)
        pitch_ratio = d_top / d_bottom if d_bottom > 1e-3 else 1.0
        yaw_ratio = d_left / d_right if d_right > 1e-3 else 1.0

        # Roll rotation angle of top edge relative to horizontal (deg)
        roll_rad = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        roll_deg = math.degrees(roll_rad)

        # Alignment status checks
        # Tolerances: centering within 10 px (~5 mm), roll within 1.5 deg, keystoning within [0.97, 1.03]
        centering_ok = (abs(delta_x_px) <= 10.0) and (abs(delta_y_px) <= 10.0)
        roll_ok = abs(roll_deg) <= 1.5
        pitch_ok = 0.97 <= pitch_ratio <= 1.03
        yaw_ok = 0.97 <= yaw_ratio <= 1.03

        is_aligned = centering_ok and roll_ok and pitch_ok and yaw_ok
        is_close = (abs(delta_x_px) <= 22.0) and (abs(delta_y_px) <= 22.0) and (abs(roll_deg) <= 3.0)

        # Directional Prompts for user
        prompts = []
        if abs(delta_x_mm) > 3.0:
            direction = "RIGHT" if delta_x_mm < 0 else "LEFT"
            prompts.append(f"PAN {direction} {abs(delta_x_mm):.1f}mm")
        if abs(delta_y_mm) > 3.0:
            direction = "DOWN / FORWARD" if delta_y_mm < 0 else "UP / BACKWARD"
            prompts.append(f"TILT {direction} {abs(delta_y_mm):.1f}mm")
        if abs(roll_deg) > 1.5:
            rot_dir = "CCW" if roll_deg > 0 else "CW"
            prompts.append(f"ROTATE {rot_dir} {abs(roll_deg):.1f}°")
        if not pitch_ok:
            pitch_dir = "FORWARD (TILT DOWN)" if pitch_ratio < 0.97 else "BACKWARD (TILT UP)"
            prompts.append(f"PITCH {pitch_dir}")

        if not prompts:
            prompts.append("PERFECTLY CENTERED & LEVELED")

        return {
            "detected": True,
            "cam_center": (cam_cx, cam_cy),
            "arena_center": (arena_cx, arena_cy),
            "delta_x_px": delta_x_px,
            "delta_y_px": delta_y_px,
            "delta_x_mm": delta_x_mm,
            "delta_y_mm": delta_y_mm,
            "scale_px_per_mm": scale_px_per_mm,
            "pitch_ratio": pitch_ratio,
            "yaw_ratio": yaw_ratio,
            "roll_deg": roll_deg,
            "centering_ok": centering_ok,
            "roll_ok": roll_ok,
            "pitch_ok": pitch_ok,
            "yaw_ok": yaw_ok,
            "is_aligned": is_aligned,
            "is_close": is_close,
            "prompts": prompts,
        }

    def render_hud(self, frame: np.ndarray, metrics: Dict[str, Any], marker_centers: Dict[int, Tuple[float, float]]) -> np.ndarray:
        """Render HUD overlays, reticles, alignment vector, and status banner."""
        hud = frame.copy()
        h, w = hud.shape[:2]

        cam_cx, cam_cy = int(metrics["cam_center"][0]), int(metrics["cam_center"][1])

        # Draw optical center reticle (White/Cyan crosshair + concentric circles)
        cv2.circle(hud, (cam_cx, cam_cy), 4, (0, 255, 255), -1)
        cv2.circle(hud, (cam_cx, cam_cy), 16, (0, 255, 255), 1)
        cv2.circle(hud, (cam_cx, cam_cy), 32, (0, 255, 255), 1)
        cv2.line(hud, (cam_cx - 45, cam_cy), (cam_cx + 45, cam_cy), (0, 255, 255), 1)
        cv2.line(hud, (cam_cx, cam_cy - 45), (cam_cx, cam_cy + 45), (0, 255, 255), 1)
        cv2.putText(hud, "CAMERA CENTER", (cam_cx + 10, cam_cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if not metrics.get("detected"):
            # Missing markers banner
            missing = metrics.get("missing_markers", [])
            cv2.rectangle(hud, (0, 0), (w, 55), (0, 0, 180), -1)
            cv2.putText(hud, f"SEARCHING FOR ARENA MARKERS... (Missing IDs: {missing})", (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            return hud

        arena_cx, arena_cy = int(metrics["arena_center"][0]), int(metrics["arena_center"][1])

        # Status color: Green if aligned, Yellow if close, Red if misaligned
        if metrics["is_aligned"]:
            status_color = (0, 255, 0)
            status_text = "STATUS: ALIGNED / LOCKED (Press [S] to Save)"
        elif metrics["is_close"]:
            status_color = (0, 255, 255)
            status_text = "STATUS: NEARLY ALIGNED (Fine Tune Stand)"
        else:
            status_color = (0, 0, 255)
            status_text = "STATUS: MISALIGNED (Adjust Stand)"

        # Draw arena boundary quadrilateral
        pts = np.array([
            marker_centers[0],
            marker_centers[1],
            marker_centers[3],
            marker_centers[2],
        ], dtype=np.int32)
        cv2.polylines(hud, [pts], isClosed=True, color=(180, 180, 180), thickness=1, lineType=cv2.LINE_AA)

        # Draw diagonals
        cv2.line(hud, (int(marker_centers[0][0]), int(marker_centers[0][1])), (int(marker_centers[3][0]), int(marker_centers[3][1])), (80, 80, 80), 1)
        cv2.line(hud, (int(marker_centers[1][0]), int(marker_centers[1][1])), (int(marker_centers[2][0]), int(marker_centers[2][1])), (80, 80, 80), 1)

        # Draw marker center points and IDs
        for mid, pt in marker_centers.items():
            cv2.circle(hud, (int(pt[0]), int(pt[1])), 5, (255, 200, 0), -1)
            cv2.putText(hud, f"M{mid}", (int(pt[0]) + 8, int(pt[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 1)

        # Draw Arena Center target bullseye
        cv2.circle(hud, (arena_cx, arena_cy), 8, status_color, 2)
        cv2.circle(hud, (arena_cx, arena_cy), 2, status_color, -1)
        cv2.putText(hud, "ARENA CENTER", (arena_cx + 10, arena_cy + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)

        # Draw alignment error vector arrow
        cv2.arrowedLine(hud, (arena_cx, arena_cy), (cam_cx, cam_cy), status_color, 2, tipLength=0.25)

        # Top Header Banner
        cv2.rectangle(hud, (0, 0), (w, 65), (20, 24, 30), -1)
        cv2.putText(hud, status_text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

        prompt_str = " | ".join(metrics["prompts"])
        cv2.putText(hud, f"GUIDANCE: {prompt_str}", (15, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1)

        # Bottom Info Bar
        cv2.rectangle(hud, (0, h - 35), (w, h), (15, 18, 22), -1)
        info_str = (
            f"Offset: dX={metrics['delta_x_mm']:+.1f}mm ({metrics['delta_x_px']:+.0f}px), "
            f"dY={metrics['delta_y_mm']:+.1f}mm ({metrics['delta_y_px']:+.0f}px) | "
            f"Roll: {metrics['roll_deg']:+.1f}° | "
            f"Pitch: {metrics['pitch_ratio']:.2f} | "
            f"[S] Save | [Space] Freeze | [Q] Quit"
        )
        cv2.putText(hud, info_str, (12, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 190, 200), 1)

        return hud


def generate_synthetic_arena_frame(
    width: int = 640,
    height: int = 480,
    offset_x_px: float = 0.0,
    offset_y_px: float = 0.0,
    tilt_pitch_ratio: float = 1.0,
    roll_deg: float = 0.0,
) -> np.ndarray:
    """Generate a clean synthetic frame containing 4 ArUco markers with configured offset and tilt."""
    img = np.full((height, width, 3), 245, dtype=np.uint8)

    # Grid lines to mimic MDF table
    for y in range(0, height, 40):
        cv2.line(img, (0, y), (width, y), (230, 230, 230), 1)
    for x in range(0, width, 40):
        cv2.line(img, (x, 0), (x, height), (230, 230, 230), 1)

    # Base nominal marker center positions in image (scaled ~1.8 px/mm)
    # Physical span: 260 mm x 140 mm -> ~468 px x 252 px
    cx = width / 2.0 - offset_x_px
    cy = height / 2.0 - offset_y_px
    half_w = 234.0
    half_h = 126.0

    # Nominal corners [Top-Left, Top-Right, Bottom-Left, Bottom-Right]
    pts = [
        np.array([-half_w * tilt_pitch_ratio, -half_h]),  # M0
        np.array([+half_w * tilt_pitch_ratio, -half_h]),  # M1
        np.array([-half_w / tilt_pitch_ratio, +half_h]),  # M2
        np.array([+half_w / tilt_pitch_ratio, +half_h]),  # M3
    ]

    # Apply roll rotation
    rad = math.radians(roll_deg)
    rot_mat = np.array([
        [math.cos(rad), -math.sin(rad)],
        [math.sin(rad), math.cos(rad)]
    ])

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_px = 42

    for mid, base_pt in enumerate(pts):
        rotated = rot_mat @ base_pt
        mx = int(round(cx + rotated[0]))
        my = int(round(cy + rotated[1]))

        # Generate marker bitmap
        if hasattr(cv2.aruco, "generateImageMarker"):
            m_img = cv2.aruco.generateImageMarker(aruco_dict, mid, marker_px)
        else:
            m_img = cv2.aruco.drawMarker(aruco_dict, mid, marker_px)

        m_bgr = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)

        # Stamp into frame
        x1 = mx - marker_px // 2
        y1 = my - marker_px // 2
        x2 = x1 + marker_px
        y2 = y1 + marker_px

        if 0 <= x1 and x2 < width and 0 <= y1 and y2 < height:
            img[y1:y2, x1:x2] = m_bgr

    return img


def run_alignment_session(
    device: str = "/dev/video0",
    synthetic: bool = False,
    save_report: bool = False,
    headless: bool = False,
) -> Dict[str, Any]:
    """Execute live or headless camera alignment session."""
    aligner = CameraAligner()

    if synthetic:
        print("[INIT] Running in Synthetic Camera Verification Mode.")
        frame = generate_synthetic_arena_frame(
            offset_x_px=3.0,
            offset_y_px=-2.0,
            tilt_pitch_ratio=1.008,
            roll_deg=0.5,
        )
        marker_centers = aligner.detect_markers(frame)
        metrics = aligner.compute_metrics(frame.shape, marker_centers)
        hud = aligner.render_hud(frame, metrics, marker_centers)

        print(f"[REPORT] Detected {len(marker_centers)}/4 markers.")
        print(f"         Centering Offset: ΔX = {metrics['delta_x_mm']:+.2f} mm, ΔY = {metrics['delta_y_mm']:+.2f} mm")
        print(f"         Roll: {metrics['roll_deg']:+.2f}°, Pitch Ratio: {metrics['pitch_ratio']:.3f}")
        print(f"         Aligned Status: {metrics['is_aligned']}")

        if save_report:
            save_alignment_data(metrics, snapshot=hud)

        return metrics

    # Physical Camera Mode
    dev_index = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(dev_index)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera device '{device}'. Falling back to synthetic verification.")
        return run_alignment_session(synthetic=True, save_report=save_report, headless=headless)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("\n" + "=" * 65)
    print("      Overhead Camera Centering & Tilt Calibration")
    print("=" * 65)
    print("  Controls:")
    print("    [s]      Save calibration report and screenshot")
    print("    [SPACE]  Freeze / Unfreeze frame")
    print("    [q/ESC]  Quit alignment tool")
    print("=" * 65 + "\n")

    frozen = False
    last_hud = None
    latest_metrics = {}

    try:
        while True:
            if not frozen:
                ret, frame = cap.read()
                if not ret:
                    print("[WARN] Failed to read frame from camera.")
                    time.sleep(0.05)
                    continue

                marker_centers = aligner.detect_markers(frame)
                latest_metrics = aligner.compute_metrics(frame.shape, marker_centers)
                hud = aligner.render_hud(frame, latest_metrics, marker_centers)
                last_hud = hud
            else:
                hud = last_hud

            if not headless:
                cv2.imshow("Camera Centering & Tilt Alignment", hud)
                key = cv2.waitKey(20) & 0xFF
                if key in [ord("q"), 27]:
                    break
                elif key == ord(" "):
                    frozen = not frozen
                    print(f"[{'PAUSED' if frozen else 'RESUMED'}] Frame display toggled.")
                elif key in [ord("s"), ord("S")]:
                    save_alignment_data(latest_metrics, snapshot=hud)
            else:
                break
    finally:
        cap.release()
        if not headless:
            cv2.destroyAllWindows()

    return latest_metrics


def save_alignment_data(metrics: Dict[str, Any], snapshot: Optional[np.ndarray] = None):
    """Save alignment results to config/camera_alignment.json and snapshot image."""
    config_dir = "config"
    data_dir = "data"
    os.makedirs(config_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)

    json_path = os.path.join(config_dir, "camera_alignment.json")
    report = {
        "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "is_aligned": metrics.get("is_aligned", False),
        "centering": {
            "delta_x_px": metrics.get("delta_x_px", 0.0),
            "delta_y_px": metrics.get("delta_y_px", 0.0),
            "delta_x_mm": metrics.get("delta_x_mm", 0.0),
            "delta_y_mm": metrics.get("delta_y_mm", 0.0),
            "scale_px_per_mm": metrics.get("scale_px_per_mm", 2.0),
        },
        "orientation": {
            "roll_deg": metrics.get("roll_deg", 0.0),
            "pitch_ratio": metrics.get("pitch_ratio", 1.0),
            "yaw_ratio": metrics.get("yaw_ratio", 1.0),
        },
        "status_checks": {
            "centering_ok": metrics.get("centering_ok", False),
            "roll_ok": metrics.get("roll_ok", False),
            "pitch_ok": metrics.get("pitch_ok", False),
            "yaw_ok": metrics.get("yaw_ok", False),
        },
        "guidance_prompts": metrics.get("prompts", []),
    }

    with open(json_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n[SAVED] Camera alignment calibration saved to: {json_path}")

    if snapshot is not None:
        snap_path = os.path.join(data_dir, "camera_alignment_snapshot.png")
        cv2.imwrite(snap_path, snapshot)
        print(f"[SAVED] Alignment snapshot written to: {snap_path}\n")


def main():
    parser = argparse.ArgumentParser(description="Overhead Camera Centering & Tilt Alignment Utility")
    parser.add_argument("--device", default="/dev/video0", help="Camera device path or index (default: /dev/video0)")
    parser.add_argument("--synthetic", action="store_true", help="Run with synthetic frame for headless / test verification")
    parser.add_argument("--save-report", action="store_true", help="Save alignment report and snapshot immediately")
    parser.add_argument("--headless", action="store_true", help="Run without opening GUI window")
    args = parser.parse_args()

    run_alignment_session(
        device=args.device,
        synthetic=args.synthetic,
        save_report=args.save_report,
        headless=args.headless,
    )


if __name__ == "__main__":
    main()
