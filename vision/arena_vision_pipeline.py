#!/usr/bin/env python3
"""
Dual xArm 1s Computer Vision & Planar Homography Pipeline.

Features:
  - Monocular camera undistortion using intrinsic matrix K and distortion coefficients D.
  - 4-point ArUco planar homography (DICT_4X4_50, IDs: 0, 1, 2, 3) at flat arena corners (Z = 0.0 mm).
  - Warps overhead camera view to metric arena grid (2.0 px/mm, 780x420 px).
  - Homography freezing/locking to guard against camera vibration and arm occlusion.
  - Multi-band HSV color segmentation (Red, Green, Blue, Yellow) with morphology filtering.
  - Moment-based centroid extraction and yaw orientation calculation in [-45°, +45°].
  - Coordinate system shifting from global arena frame (Xw, Yw) to arm polar frames (r, theta).
  - Full compatibility with both OpenCV >= 4.7 and OpenCV < 4.7.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import yaml


@dataclass
class DetectedCube:
    """Represents a physical colored wooden cube detected on the picking arena."""

    color: str                        # "red", "green", "blue", "yellow"
    world_x: float                    # Global arena X in mm [85.0, 475.0]
    world_y: float                    # Global arena Y in mm [45.0, 255.0]
    world_z: float = 12.5             # Tabletop elevation + half cube height (mm)
    yaw_deg: float = 0.0              # Planar cube rotation angle in [-45.0, +45.0] deg
    assigned_arm: str = "xarm_2"      # "xarm_1" (Left) or "xarm_2" (Right)
    
    # Polar coordinates relative to xArm 1 (Left: X=170, Y=360, Heading=-110°)
    r1_mm: float = 0.0
    theta1_deg: float = 0.0
    
    # Polar coordinates relative to xArm 2 (Right: X=390, Y=360, Heading=-70°)
    r2_mm: float = 0.0
    theta2_deg: float = 0.0
    
    # Polar coordinates for assigned arm
    target_r_mm: float = 0.0
    target_theta_deg: float = 0.0
    
    # Spatial mutex requirement (True if within central overlap corridor X in [240, 320])
    requires_mutex: bool = False
    
    # Contour bounding box & center in rectified pixels
    pixel_center: Tuple[float, float] = (0.0, 0.0)
    bounding_box: Optional[np.ndarray] = None
    contour_area_px: float = 0.0
    confidence: float = 1.0


class ArenaVisionPipeline:
    """Core perception pipeline for the Dual xArm 1s color-sorting workcell."""

    def __init__(self, config_dir: str = "config", scale_px_per_mm: float = 2.0):
        self.config_dir = config_dir
        self.scale_px_per_mm = scale_px_per_mm

        # ArUco Reference Markers Ground Truth Coordinates (mm at Z = 0.0 mm)
        self.marker_world_corners: Dict[int, Tuple[float, float]] = {
            0: (85.0, 255.0),   # Marker 0: Rear-Left
            1: (475.0, 255.0),  # Marker 1: Rear-Right
            2: (85.0, 45.0),    # Marker 2: Front-Left
            3: (475.0, 45.0),   # Marker 3: Front-Right
        }

        # Arena physical dimensions between marker centers
        self.arena_width_mm = 475.0 - 85.0   # 390.0 mm
        self.arena_height_mm = 255.0 - 45.0  # 210.0 mm

        # Rectified metric image dimensions in pixels
        self.rect_w = int(round(self.arena_width_mm * self.scale_px_per_mm))    # 780 px
        self.rect_h = int(round(self.arena_height_mm * self.scale_px_per_mm))   # 420 px

        # Destination points in rectified image plane (Origin (0,0) is top-left: Marker 0)
        self.dst_pts = np.array([
            [0.0, 0.0],                          # ID 0 (Rear-Left)
            [float(self.rect_w), 0.0],           # ID 1 (Rear-Right)
            [0.0, float(self.rect_h)],           # ID 2 (Front-Left)
            [float(self.rect_w), float(self.rect_h)],  # ID 3 (Front-Right)
        ], dtype=np.float32)

        # Base mounts and nominal headings in world frame (from dual_xarm_hardware.yaml)
        self.base_mounts = {
            "xarm_1": {"x": 170.0, "y": 360.0, "z": 0.0, "heading": -110.0},
            "xarm_2": {"x": 390.0, "y": 360.0, "z": 0.0, "heading": -70.0},
        }
        self.midline_x_mm = 280.0
        self.mutex_corridor = (240.0, 320.0)

        # Camera Intrinsic Parameters
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.has_calibration: bool = False

        # Homography Matrix
        self.H_matrix: Optional[np.ndarray] = None
        self.H_locked: bool = False

        # Color and morphology configuration
        self.color_config: Dict = {}
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Setup ArUco Detector with backward compatibility
        self._init_aruco_detector()

        # Load configurations
        self.load_camera_calibration()
        self.load_color_config()

    def _init_aruco_detector(self):
        """Initialize ArUco detector supporting both OpenCV >= 4.7 and older versions."""
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Check if newer ArucoDetector class is available (OpenCV >= 4.7.0)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._use_new_aruco_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_detector = None
            self._use_new_aruco_api = False

    def load_camera_calibration(self, filename: Optional[str] = None):
        """Loads intrinsic matrix and distortion coefficients from YAML."""
        if filename is None:
            filename = os.path.join(self.config_dir, "camera_calibration.yaml")

        if not os.path.exists(filename):
            return

        try:
            with open(filename, "r") as f:
                data = yaml.safe_load(f)

            if not data:
                return

            cam_mat = data.get("camera_matrix", {}).get("data", None)
            if cam_mat and len(cam_mat) == 9:
                self.camera_matrix = np.array(cam_mat, dtype=np.float32).reshape((3, 3))

            dist = data.get("distortion_coefficients", {}).get("data", None)
            if dist:
                self.dist_coeffs = np.array(dist, dtype=np.float32).reshape((1, -1))

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                self.has_calibration = True

            # Check if homography is already cached
            h_data = data.get("planar_homography", {}).get("matrix", {}).get("data", None)
            if h_data and len(h_data) == 9:
                cand_h = np.array(h_data, dtype=np.float32).reshape((3, 3))
                # Only use if not identity
                if not np.allclose(cand_h, np.eye(3)):
                    self.H_matrix = cand_h

        except Exception as e:
            print(f"[ArenaVisionPipeline] Warning loading camera calibration: {e}")

    def load_color_config(self, filename: Optional[str] = None):
        """Loads HSV segmentation ranges and morphological thresholds."""
        if filename is None:
            filename = os.path.join(self.config_dir, "colors.yaml")

        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    self.color_config = yaml.safe_load(f) or {}
            except Exception as e:
                print(f"[ArenaVisionPipeline] Error loading colors.yaml: {e}")

        # Fallback default colors if file not present or incomplete
        if "colors" not in self.color_config:
            self.color_config["colors"] = {
                "red": {
                    "name": "Red",
                    "bgr_display": [0, 0, 255],
                    "bands": [
                        {"h_min": 0, "h_max": 10, "s_min": 120, "s_max": 255, "v_min": 70, "v_max": 255},
                        {"h_min": 170, "h_max": 180, "s_min": 120, "s_max": 255, "v_min": 70, "v_max": 255},
                    ],
                },
                "green": {
                    "name": "Green",
                    "bgr_display": [0, 255, 0],
                    "bands": [
                        {"h_min": 38, "h_max": 85, "s_min": 100, "s_max": 255, "v_min": 60, "v_max": 255},
                    ],
                },
                "blue": {
                    "name": "Blue",
                    "bgr_display": [255, 0, 0],
                    "bands": [
                        {"h_min": 98, "h_max": 130, "s_min": 120, "s_max": 255, "v_min": 60, "v_max": 255},
                    ],
                },
                "yellow": {
                    "name": "Yellow",
                    "bgr_display": [0, 255, 255],
                    "bands": [
                        {"h_min": 20, "h_max": 36, "s_min": 120, "s_max": 255, "v_min": 100, "v_max": 255},
                    ],
                },
            }

        if "morphology" not in self.color_config:
            self.color_config["morphology"] = {
                "kernel_size": 5,
                "open_iterations": 1,
                "close_iterations": 2,
                "min_area_px": 1400,
                "max_area_px": 3800,
                "max_aspect_ratio": 1.45,
            }

        k_size = self.color_config.get("morphology", {}).get("kernel_size", 5)
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        """Undistorts raw camera frame using calibrated intrinsic matrix and coefficients."""
        if not self.has_calibration or self.camera_matrix is None or self.dist_coeffs is None:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

    def detect_aruco_markers(self, frame: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Detects ArUco markers and returns corners and IDs."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._use_new_aruco_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids

    def calibrate_homography(self, frame: np.ndarray, force_update: bool = False) -> bool:
        """
        Detects 4 arena ArUco markers (0, 1, 2, 3) and computes the planar homography matrix H.
        Returns True if all 4 markers are found and H is successfully calculated.
        """
        if self.H_locked and not force_update and self.H_matrix is not None:
            return True

        corners, ids = self.detect_aruco_markers(frame)
        if ids is None or len(ids) < 4:
            return False

        detected_ids = ids.flatten()
        if not all(marker_id in detected_ids for marker_id in [0, 1, 2, 3]):
            return False

        src_pts = np.zeros((4, 2), dtype=np.float32)
        for i, marker_id in enumerate([0, 1, 2, 3]):
            idx = np.where(detected_ids == marker_id)[0][0]
            # Center of the 40mm marker
            marker_corners = corners[idx][0]
            center = np.mean(marker_corners, axis=0)
            src_pts[i] = center

        self.H_matrix = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
        return True

    def lock_homography(self, locked: bool = True):
        """Freezes or unlocks the homography matrix to prevent drift or jitter."""
        self.H_locked = locked

    def warp_arena(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Warps the full camera image into the rectified metric arena grid (780x420 px)."""
        if self.H_matrix is None:
            if not self.calibrate_homography(frame):
                return None
        return cv2.warpPerspective(frame, self.H_matrix, (self.rect_w, self.rect_h))

    def pixel_to_world(self, u: float, v: float) -> Tuple[float, float, float]:
        """
        Maps rectified arena image pixel (u, v) to physical world millimeter coordinates (Xw, Yw, Zw).
        u=0 is Marker 0 (X=85.0), u=780 is Marker 1 (X=475.0).
        v=0 is Marker 0 (Y=255.0), v=420 is Marker 2 (Y=45.0).
        """
        x_w = 85.0 + (u / self.scale_px_per_mm)
        y_w = 255.0 - (v / self.scale_px_per_mm)
        z_w = 12.5  # Tabletop surface Z=0.0 mm + half cube height 12.5 mm
        return (round(x_w, 2), round(y_w, 2), z_w)

    def world_to_pixel(self, x_w: float, y_w: float) -> Tuple[int, int]:
        """Maps physical world millimeters (Xw, Yw) back to rectified pixel (u, v)."""
        u = int(round((x_w - 85.0) * self.scale_px_per_mm))
        v = int(round((255.0 - y_w) * self.scale_px_per_mm))
        return (u, v)

    def world_to_arm_polar(self, x_w: float, y_w: float, arm_id: str) -> Tuple[float, float]:
        """
        Transforms global world coordinates (Xw, Yw) into arm-relative cylindrical/polar coordinates (r, theta_arm).
        
        Formula:
          DeltaX = Xw - X_base, DeltaY = Yw - Y_base
          r = sqrt(DeltaX² + DeltaY²)
          theta_world = atan2(DeltaY, DeltaX) in degrees
          theta_arm = theta_world - theta_heading (normalized to [-180°, +180°])
        """
        arm_info = self.base_mounts.get(arm_id, self.base_mounts["xarm_1"])
        bx = arm_info["x"]
        by = arm_info["y"]
        heading = arm_info["heading"]

        dx = x_w - bx
        dy = y_w - by
        r = math.hypot(dx, dy)
        theta_world = math.degrees(math.atan2(dy, dx))
        
        # Relative angle from arm nominal bisector heading
        theta_arm = theta_world - heading
        # Normalize to [-180.0, +180.0]
        theta_arm = (theta_arm + 180.0) % 360.0 - 180.0

        return (round(r, 2), round(theta_arm, 2))

    def detect_cubes(self, warped_frame: np.ndarray) -> List[DetectedCube]:
        """
        Segments colored cubes in the rectified arena frame and computes their world & polar coordinates.
        """
        hsv = cv2.cvtColor(warped_frame, cv2.COLOR_BGR2HSV)
        detected_cubes: List[DetectedCube] = []

        morph_cfg = self.color_config.get("morphology", {})
        open_iters = morph_cfg.get("open_iterations", 1)
        close_iters = morph_cfg.get("close_iterations", 2)
        min_area = morph_cfg.get("min_area_px", 1400)
        max_area = morph_cfg.get("max_area_px", 3800)
        max_aspect = morph_cfg.get("max_aspect_ratio", 1.45)

        colors_dict = self.color_config.get("colors", {})

        for color_key, color_info in colors_dict.items():
            bands = color_info.get("bands", [])
            if not bands:
                continue

            # Combine sub-bands (e.g. for Red dual hue)
            combined_mask = np.zeros(warped_frame.shape[:2], dtype=np.uint8)
            for band in bands:
                lower = np.array([band["h_min"], band["s_min"], band["v_min"]], dtype=np.uint8)
                upper = np.array([band["h_max"], band["s_max"], band["v_max"]], dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                combined_mask = cv2.bitwise_or(combined_mask, mask)

            # Morphological cleaning
            if open_iters > 0:
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=open_iters)
            if close_iters > 0:
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=close_iters)

            # Find contours
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (min_area <= area <= max_area):
                    continue

                # Minimum area bounding rectangle for yaw orientation
                rect = cv2.minAreaRect(cnt)
                (cx, cy), (w_box, h_box), angle = rect

                # Reject highly elongated or non-square contours
                aspect = max(w_box, h_box) / max(min(w_box, h_box), 1e-4)
                if aspect > max_aspect:
                    continue

                # Refine centroid with spatial moments if valid
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    u_c = M["m10"] / M["m00"]
                    v_c = M["m01"] / M["m00"]
                else:
                    u_c, v_c = cx, cy

                # Normalize yaw angle to [-45°, +45°]
                yaw = angle - 90.0 if w_box < h_box else angle
                while yaw < -45.0:
                    yaw += 90.0
                while yaw > 45.0:
                    yaw -= 90.0

                # Convert pixel coordinates to physical arena world frame
                x_w, y_w, z_w = self.pixel_to_world(u_c, v_c)

                # Polar coordinates for xArm 1
                r1, th1 = self.world_to_arm_polar(x_w, y_w, "xarm_1")
                # Polar coordinates for xArm 2
                r2, th2 = self.world_to_arm_polar(x_w, y_w, "xarm_2")

                # Territory assignment: X <= 280 mm -> xArm 1, X > 280 mm -> xArm 2
                assigned_arm = "xarm_1" if x_w <= self.midline_x_mm else "xarm_2"
                active_r = r1 if assigned_arm == "arm_1" or assigned_arm == "xarm_1" else r2
                active_th = th1 if assigned_arm == "arm_1" or assigned_arm == "xarm_1" else th2

                # Central shared corridor flag
                requires_mutex = (self.mutex_corridor[0] <= x_w <= self.mutex_corridor[1])

                # Reach envelope check (must be reachable by active arm: 120 mm <= r <= 320 mm)
                if 115.0 <= active_r <= 325.0:
                    box_points = cv2.boxPoints(rect).astype(np.int32)
                    cube = DetectedCube(
                        color=color_key,
                        world_x=x_w,
                        world_y=y_w,
                        world_z=z_w,
                        yaw_deg=round(yaw, 2),
                        assigned_arm=assigned_arm,
                        r1_mm=r1,
                        theta1_deg=th1,
                        r2_mm=r2,
                        theta2_deg=th2,
                        target_r_mm=active_r,
                        target_theta_deg=active_th,
                        requires_mutex=requires_mutex,
                        pixel_center=(round(u_c, 1), round(v_c, 1)),
                        bounding_box=box_points,
                        contour_area_px=float(area),
                    )
                    detected_cubes.append(cube)

        return detected_cubes

    def annotate_frame(
        self,
        warped_frame: np.ndarray,
        cubes: List[DetectedCube],
        draw_grid: bool = True,
    ) -> np.ndarray:
        """Draws metric millimeter grid, cube bounding boxes, color tags, and coordinates."""
        annotated = warped_frame.copy()

        # 1. Draw metric grid (50 mm intervals)
        if draw_grid:
            # Vertical lines (every 50 mm from X=100 to X=450)
            for x_mm in range(100, 475, 50):
                u, _ = self.world_to_pixel(float(x_mm), 150.0)
                if 0 <= u < self.rect_w:
                    cv2.line(annotated, (u, 0), (u, self.rect_h), (60, 60, 60), 1)
                    cv2.putText(
                        annotated, f"{x_mm}", (u + 2, self.rect_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA
                    )

            # Horizontal lines (every 50 mm from Y=50 to Y=250)
            for y_mm in range(50, 260, 50):
                _, v = self.world_to_pixel(280.0, float(y_mm))
                if 0 <= v < self.rect_h:
                    cv2.line(annotated, (0, v), (self.rect_w, v), (60, 60, 60), 1)
                    cv2.putText(
                        annotated, f"{y_mm}", (4, v - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA
                    )

            # Central midline (X = 280 mm) in bright cyan dashed/solid line
            mid_u, _ = self.world_to_pixel(self.midline_x_mm, 150.0)
            cv2.line(annotated, (mid_u, 0), (mid_u, self.rect_h), (255, 255, 0), 2)
            cv2.putText(
                annotated, "MIDLINE X=280", (mid_u - 45, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA
            )

        # 2. Draw detected cubes
        for cube in cubes:
            color_bgr = self.color_config.get("colors", {}).get(cube.color, {}).get("bgr_display", [0, 255, 255])
            
            # Draw rotated bounding box
            if cube.bounding_box is not None:
                cv2.drawContours(annotated, [cube.bounding_box], 0, color_bgr, 2)

            # Draw centroid point
            uc, vc = int(round(cube.pixel_center[0])), int(round(cube.pixel_center[1]))
            cv2.circle(annotated, (uc, vc), 4, (0, 0, 0), -1)
            cv2.circle(annotated, (uc, vc), 3, (255, 255, 255), -1)

            # Draw orientation arrow
            arrow_len = 16.0
            rad = math.radians(cube.yaw_deg)
            ex = int(round(uc + arrow_len * math.cos(rad)))
            ey = int(round(vc - arrow_len * math.sin(rad)))
            cv2.arrowedLine(annotated, (uc, vc), (ex, ey), (0, 0, 255), 2, tipLength=0.3)

            # Label text
            tag = f"{cube.color.upper()} ({cube.assigned_arm})"
            coords = f"W:({cube.world_x:.0f},{cube.world_y:.0f}) r:{cube.target_r_mm:.0f}mm th:{cube.target_theta_deg:+.0f}°"
            
            # Place text above cube
            text_y = max(vc - 16, 20)
            cv2.putText(annotated, tag, (uc - 30, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, tag, (uc - 30, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated, coords, (uc - 50, text_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, coords, (uc - 50, text_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (220, 220, 220), 1, cv2.LINE_AA)

        return annotated

    def process_frame(
        self, frame: np.ndarray, draw_grid: bool = True
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], List[DetectedCube]]:
        """
        Full perception pipeline:
          1. Undistort camera frame
          2. Warp to metric arena grid
          3. Detect colored cubes and calculate coordinates
          4. Return (warped_raw, annotated_frame, detected_cubes)
        """
        undistorted = self.undistort(frame)
        warped = self.warp_arena(undistorted)
        if warped is None:
            return None, None, []

        cubes = self.detect_cubes(warped)
        annotated = self.annotate_frame(warped, cubes, draw_grid=draw_grid)
        return warped, annotated, cubes
