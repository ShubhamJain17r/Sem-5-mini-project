#!/usr/bin/env python3
"""
Single xArm 1S Computer Vision & Planar Homography Pipeline.

Features:
  - Monocular camera undistortion using intrinsic matrix K and distortion coefficients D.
  - 4-point ArUco planar homography (DICT_4X4_50, IDs: 0, 1, 2, 3) at flat arena corners (Z = 0.0 mm).
  - Warps overhead camera view to metric arena grid (2.0 px/mm, 520x280 px).
  - Homography freezing/locking to guard against camera vibration and arm occlusion.
  - Multi-band HSV color segmentation (Red, Green, Blue, Yellow) with morphology filtering.
  - Moment-based centroid extraction and yaw orientation calculation in [-45°, +45°].
  - Coordinate system shifting from global arena frame (Xw, Yw) to arm polar frame (r, theta).
  - Full compatibility with both OpenCV >= 4.7 and OpenCV < 4.7.
"""

from dataclasses import dataclass, field
import math
import os
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import yaml


@dataclass
class DetectedCube:
    """Represents a physical colored wooden cube detected on the picking arena."""

    color: str                        # "red", "green", "blue", "yellow"
    world_x: float                    # Global arena X in mm [70.0, 330.0]
    world_y: float                    # Global arena Y in mm [40.0, 180.0]
    world_z: float = 9.53             # Tabletop elevation + half block height (mm)
    yaw_deg: float = 0.0              # Planar cube rotation angle in [-45.0, +45.0] deg
    assigned_arm: str = "xarm_1"      # Primary manipulator
    
    # Polar coordinates relative to single xArm 1S (Base: X=200, Y=280, Heading=-90°)
    r_mm: float = 0.0
    theta_deg: float = 0.0
    target_r_mm: float = 0.0
    target_theta_deg: float = 0.0
    
    # Backward-compatibility aliases
    r1_mm: float = 0.0
    theta1_deg: float = 0.0
    r2_mm: float = 0.0
    theta2_deg: float = 0.0
    requires_mutex: bool = False
    
    # Contour bounding box & center in rectified pixels
    pixel_center: Tuple[float, float] = (0.0, 0.0)
    bounding_box: Optional[np.ndarray] = None
    contour_area_px: float = 0.0
    confidence: float = 1.0
    targetable: bool = True

    # Orientation, dimensions and wrist roll alignment
    angle_deg: float = 0.0
    width_px: float = 0.0
    height_px: float = 0.0
    gripper_roll_tick: int = 500


class ArenaVisionPipeline:
    """Core perception pipeline for the Single xArm 1S color-sorting workcell."""

    def __init__(self, config_dir: str = "config", scale_px_per_mm: float = 2.0):
        self.config_dir = config_dir
        self.scale_px_per_mm = scale_px_per_mm

        # ArUco Reference Markers Ground Truth Coordinates (mm at Z = 0.0 mm)
        # Arena: 300 mm (W) x 250 mm (D) between marker centers in Rows 6..11 (Rows 3..5 excluded)
        self.marker_world_corners: Dict[int, Tuple[float, float]] = {
            0: (25.0, 275.0),   # Marker 0 / Top-Left (Row 6, Col 0)
            1: (325.0, 275.0),  # Marker 1 / Top-Right (Row 6, Col 6)
            2: (25.0, 25.0),    # Marker 2 / Bottom-Left (Row 11, Col 0)
            3: (325.0, 25.0),   # Marker 3 / Bottom-Right (Row 11, Col 6)
        }

        # Arena physical dimensions between marker centers
        self.arena_width_mm = 325.0 - 25.0   # 300.0 mm
        self.arena_height_mm = 275.0 - 25.0  # 250.0 mm

        # Rectified metric image dimensions in pixels
        self.rect_w = int(round(self.arena_width_mm * self.scale_px_per_mm))    # 600 px
        self.rect_h = int(round(self.arena_height_mm * self.scale_px_per_mm))   # 500 px

        # Destination points in rectified image plane (Top-Left -> Top-Right -> Bottom-Right -> Bottom-Left)
        self.dst_pts = np.array([
            [0.0, 0.0],                                # Top-Left (Row 6, Col 0)
            [float(self.rect_w), 0.0],                 # Top-Right (Row 6, Col 6)
            [float(self.rect_w), float(self.rect_h)],  # Bottom-Right (Row 11, Col 6)
            [0.0, float(self.rect_h)],                 # Bottom-Left (Row 11, Col 0)
        ], dtype=np.float32)

        # Base mount and nominal heading in world frame (from xarm_hardware.yaml)
        self.base_mounts = {
            "xarm_1": {"x": 175.0, "y": 375.0, "z": 0.0, "heading": -90.0},
        }
        self.midline_x_mm = 175.0
        self.mutex_corridor = (0.0, 0.0)  # No mutex corridor needed for single arm

        # Camera Intrinsic Parameters & Extrinsics Telemetry
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.calib_width: int = 1920
        self.calib_height: int = 1080
        self.has_calibration: bool = False
        self.camera_extrinsics: Optional[Dict[str, Any]] = None

        # Homography Matrix
        self.H_matrix: Optional[np.ndarray] = None
        self.H_locked: bool = False

        # Color and morphology configuration
        self.color_config: Dict = {}
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.cube_size_mm: float = 37.5
        self.cube_height_mm: float = 45.0
        self.cube_center_z_mm: float = 22.5
        self.marker_size_mm: float = 50.0

        # Setup ArUco Detector with backward compatibility
        self._init_aruco_detector()

        # Load configurations
        self.load_camera_calibration()
        self.load_color_config()
        self.load_hardware_config()

    def _init_aruco_detector(self):
        """Initialize ArUco detector supporting both OpenCV >= 4.7 and older versions."""
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Check if newer ArucoDetector class is available (OpenCV >= 4.7.0)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.adaptiveThreshWinSizeStep = 2
            self.aruco_params.minMarkerPerimeterRate = 0.01
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self._use_new_aruco_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.aruco_params.adaptiveThreshWinSizeMin = 3
            self.aruco_params.adaptiveThreshWinSizeMax = 23
            self.aruco_params.adaptiveThreshWinSizeStep = 2
            self.aruco_params.minMarkerPerimeterRate = 0.01
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
            self.calib_width = int(data.get("image_width", 1920))
            self.calib_height = int(data.get("image_height", 1080))

            dist = data.get("distortion_coefficients", {}).get("data", None)
            if dist:
                self.dist_coeffs = np.array(dist, dtype=np.float32).reshape((1, -1))

            if self.camera_matrix is not None and self.dist_coeffs is not None:
                self.has_calibration = True

            # Load marker corners if specified in YAML
            markers_yaml = data.get("aruco_reference_markers", {}).get("markers", {})
            for mkey, mval in markers_yaml.items():
                mid = int(mval.get("id", -1))
                if mid in [0, 1, 2, 3]:
                    self.marker_world_corners[mid] = (float(mval["world_x"]), float(mval["world_y"]))

            # Recompute dimensions from loaded corners
            all_x = [pt[0] for pt in self.marker_world_corners.values()]
            all_y = [pt[1] for pt in self.marker_world_corners.values()]
            if all_x and all_y:
                self.arena_width_mm = max(all_x) - min(all_x)
                self.arena_height_mm = max(all_y) - min(all_y)
                self.rect_w = int(round(self.arena_width_mm * self.scale_px_per_mm))
                self.rect_h = int(round(self.arena_height_mm * self.scale_px_per_mm))
                self.dst_pts = np.array([
                    [0.0, 0.0],
                    [float(self.rect_w), 0.0],
                    [float(self.rect_w), float(self.rect_h)],
                    [0.0, float(self.rect_h)],
                ], dtype=np.float32)

            # Check if homography is already cached
            h_data = data.get("planar_homography", {}).get("matrix", {}).get("data", None)
            if h_data and len(h_data) == 9:
                cand_h = np.array(h_data, dtype=np.float32).reshape((3, 3))
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
                "min_area_px": 1000,
                "max_area_px": 25000,
                "max_aspect_ratio": 2.5,
            }

        k_size = self.color_config.get("morphology", {}).get("kernel_size", 5)
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))

    def load_hardware_config(self, filename: Optional[str] = None):
        """Loads picking arena parameters such as cube_center_z_mm from xarm_hardware.yaml."""
        if filename is None:
            filename = os.path.join(self.config_dir, "xarm_hardware.yaml")

        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    data = yaml.safe_load(f) or {}
                arena_cfg = data.get("workspace", {}).get("picking_arena", {})
                if "cube_size_mm" in arena_cfg:
                    self.cube_size_mm = float(arena_cfg["cube_size_mm"])
                if "cube_height_mm" in arena_cfg:
                    self.cube_height_mm = float(arena_cfg["cube_height_mm"])
                if "cube_center_z_mm" in arena_cfg:
                    self.cube_center_z_mm = float(arena_cfg["cube_center_z_mm"])
                marker_cfg = data.get("workspace", {}).get("aruco_markers", {})
                if "marker_size_mm" in marker_cfg:
                    self.marker_size_mm = float(marker_cfg["marker_size_mm"])
                arm_mount = data.get("manipulators", {}).get("xarm_1", {}).get("base_mount_mm", {})
                if arm_mount:
                    self.base_mounts["xarm_1"]["x"] = float(arm_mount.get("x", self.base_mounts["xarm_1"]["x"]))
                    self.base_mounts["xarm_1"]["y"] = float(arm_mount.get("y", self.base_mounts["xarm_1"]["y"]))
                    self.base_mounts["xarm_1"]["z"] = float(arm_mount.get("z", self.base_mounts["xarm_1"]["z"]))
                    self.base_mounts["xarm_1"]["heading"] = float(arm_mount.get("nominal_heading_deg", self.base_mounts["xarm_1"]["heading"]))
                    self.midline_x_mm = self.base_mounts["xarm_1"]["x"]
            except Exception as e:
                print(f"[ArenaVisionPipeline] Warning loading hardware config: {e}")

    def _get_scaled_intrinsics(self, frame_shape: Tuple[int, ...]) -> Tuple[np.ndarray, np.ndarray]:
        """Scale calibrated 1920x1080 intrinsics proportionally to match active frame dimensions."""
        h, w = frame_shape[:2]
        if self.camera_matrix is not None and self.calib_width > 0 and self.calib_height > 0:
            sy = float(h) / float(self.calib_height)
            sx = float(w) / float(self.calib_width)
            # Standard 4:3 (640x480) mode crops horizontally from 16:9 (1920x1080), preserving vertical FOV
            f_scale = sy if abs((w / max(h, 1)) - (self.calib_width / max(self.calib_height, 1))) > 0.1 else sx
            K = self.camera_matrix.astype(np.float64).copy()
            K[0, 0] *= f_scale
            K[1, 1] *= f_scale
            K[0, 2] *= sx
            K[1, 2] *= sy
            dist = self.dist_coeffs.astype(np.float64) if self.dist_coeffs is not None else np.zeros(5)
            return K, dist

        K = np.array([
            [float(w), 0.0, float(w) / 2.0],
            [0.0, float(w), float(h) / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        return K, np.zeros(5)

    def undistort(self, frame: np.ndarray) -> np.ndarray:
        """Undistorts raw camera frame using resolution-scaled intrinsic matrix and coefficients."""
        if not self.has_calibration or self.camera_matrix is None or self.dist_coeffs is None:
            return frame
        K, dist = self._get_scaled_intrinsics(frame.shape)
        return cv2.undistort(frame, K, dist)

    def detect_aruco_markers(self, frame: np.ndarray) -> Tuple[Optional[List[np.ndarray]], Optional[np.ndarray]]:
        """Detects ArUco markers and returns corners and IDs with edge padding to preserve quiet zones."""
        pad = 20
        padded = cv2.copyMakeBorder(frame, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=[255, 255, 255])
        gray = cv2.cvtColor(padded, cv2.COLOR_BGR2GRAY)
        if self._use_new_aruco_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if corners is None or len(corners) < 4:
            # Fallback with CLAHE for low-contrast marker borders or uneven illumination
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            cl_gray = clahe.apply(gray)
            if self._use_new_aruco_api:
                c_cl, ids_cl, _ = self.aruco_detector.detectMarkers(cl_gray)
            else:
                c_cl, ids_cl, _ = cv2.aruco.detectMarkers(
                    cl_gray, self.aruco_dict, parameters=self.aruco_params
                )
            if ids_cl is not None and (ids is None or len(ids_cl) > len(ids)):
                corners, ids = c_cl, ids_cl

        if corners is not None and len(corners) > 0:
            unpadded_corners = [c - pad for c in corners]
            return unpadded_corners, ids
        return corners, ids

    def calibrate_homography(self, frame: np.ndarray, force_update: bool = False) -> bool:
        """Detects 4 arena ArUco markers (0, 1, 2, 3) and computes planar homography matrix H."""
        if self.H_locked and not force_update and self.H_matrix is not None:
            return True

        corners, ids = self.detect_aruco_markers(frame)
        if ids is None or len(ids) < 4:
            return self.H_matrix is not None

        detected_ids = ids.flatten()
        if not all(marker_id in detected_ids for marker_id in [0, 1, 2, 3]):
            return self.H_matrix is not None

        id_to_center = {}
        for i, mid in enumerate(detected_ids):
            id_to_center[int(mid)] = np.mean(corners[i][0], axis=0)

        # Exact ID mapping:
        # Marker 0: Top-Left (Row 6, Col 0) -> dst_pts[0]
        # Marker 1: Top-Right (Row 6, Col 6) -> dst_pts[1]
        # Marker 3: Bottom-Right (Row 11, Col 6) -> dst_pts[2]
        # Marker 2: Bottom-Left (Row 11, Col 0) -> dst_pts[3]
        if all(k in id_to_center for k in [0, 1, 3, 2]):
            tl = id_to_center[0]
            tr = id_to_center[1]
            br = id_to_center[3]
            bl = id_to_center[2]
        else:
            centers = np.array([np.mean(c[0], axis=0) for c in corners[:4]], dtype=np.float32)
            sorted_by_y = centers[np.argsort(centers[:, 1])]
            top_2 = sorted_by_y[:2]
            bot_2 = sorted_by_y[2:]
            tl = top_2[np.argmin(top_2[:, 0])]
            tr = top_2[np.argmax(top_2[:, 0])]
            bl = bot_2[np.argmin(bot_2[:, 0])]
            br = bot_2[np.argmax(bot_2[:, 0])]

        src_pts = np.array([tl, tr, br, bl], dtype=np.float32)
        self.H_matrix = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
        return True

    def compute_camera_extrinsics(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Computes 3D camera extrinsics (position, height Hc, distance Dc, depression angle, side)
        using cv2.solvePnP against the 4 ground-truth ArUco marker coordinates.
        """
        corners, ids = self.detect_aruco_markers(frame)
        if ids is None or len(ids) < 4:
            return {
                "success": False,
                "message": f"Requires all 4 markers, found {len(ids) if ids is not None else 0}",
            }

        id_to_center = {}
        for i, mid in enumerate(ids.flatten()):
            id_to_center[int(mid)] = np.mean(corners[i][0], axis=0)

        if not all(k in id_to_center for k in [0, 1, 3, 2]):
            missing = [k for k in [0, 1, 3, 2] if k not in id_to_center]
            return {
                "success": False,
                "message": f"Missing required markers: {missing}",
            }

        object_pts = np.array([
            [25.0, 275.0, 0.0],   # M0 (Row 6, Col 0)
            [325.0, 275.0, 0.0],  # M1 (Row 6, Col 6)
            [325.0, 25.0, 0.0],   # M3 (Row 11, Col 6)
            [25.0, 25.0, 0.0],    # M2 (Row 11, Col 0)
        ], dtype=np.float64)

        image_pts = np.array([
            id_to_center[0],
            id_to_center[1],
            id_to_center[3],
            id_to_center[2],
        ], dtype=np.float64)

        K, dist = self._get_scaled_intrinsics(frame.shape)

        # Cross-check raw pixels/mm ratio from individual physical 50.0mm (5.0 cm) ArUco markers
        marker_px_sides = []
        for i, mid in enumerate(ids.flatten()):
            if int(mid) in [0, 1, 2, 3]:
                pts = corners[i][0]
                side_px = 0.25 * (
                    float(np.linalg.norm(pts[1] - pts[0])) + float(np.linalg.norm(pts[2] - pts[1])) +
                    float(np.linalg.norm(pts[3] - pts[2])) + float(np.linalg.norm(pts[0] - pts[3]))
                )
                marker_px_sides.append(side_px)
        raw_px_per_mm = (sum(marker_px_sides) / len(marker_px_sides)) / max(self.marker_size_mm, 1.0) if marker_px_sides else 0.0

        ret, rvec, tvec = cv2.solvePnP(object_pts, image_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ret:
            return {"success": False, "message": "solvePnP failed to converge"}

        R, _ = cv2.Rodrigues(rvec)
        cam_pos_world = (-R.T @ tvec).flatten()
        optical_axis_world = (R.T @ np.array([0, 0, 1.0])).flatten()
        
        # Depression angle (pitch below horizontal plane)
        sin_dep = max(-1.0, min(1.0, -optical_axis_world[2]))
        depression_deg = float(np.degrees(np.arcsin(sin_dep)))

        arena_center = np.array([175.0, 150.0, 0.0])
        dist_to_center = float(np.linalg.norm(cam_pos_world - arena_center))

        x_c, y_c = cam_pos_world[0], cam_pos_world[1]
        if x_c > 350.0:
            side = "East / Right (facing West toward arena)"
        elif x_c < 0.0:
            side = "West / Left (facing East toward arena)"
        elif y_c < 0.0:
            side = "South / Front (looking North toward manipulator)"
        else:
            side = "North / Rear (behind manipulator)"

        result = {
            "success": True,
            "camera_position_world_mm": {
                "x": round(float(cam_pos_world[0]), 1),
                "y": round(float(cam_pos_world[1]), 1),
                "z": round(float(cam_pos_world[2]), 1),
            },
            "height_mm": round(float(cam_pos_world[2]), 1),
            "distance_to_arena_center_mm": round(dist_to_center, 1),
            "depression_angle_deg": round(depression_deg, 1),
            "raw_px_per_mm": round(raw_px_per_mm, 3),
            "viewing_side": side,
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
        }
        self.camera_extrinsics = result
        return result

    def verify_aruco_markers(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Validates presence, corner clearances, and physical orientation angles of all 4 markers:
        - M0 at (Row 6, Col 0 / Top-Left): pointing West (~180°)
        - M1 at (Row 6, Col 6 / Top-Right): pointing West (~180°)
        - M2 at (Row 11, Col 0 / Bottom-Left): pointing East (~0°)
        - M3 at (Row 11, Col 6 / Bottom-Right): pointing East (~0°)
        Returns diagnostic reports with actionable adjustment guidance.
        """
        specs = {
            0: {"name": "Marker 0 (Row 6, Col 0 / Top-Left)", "expected_deg": 180.0, "expected_dir": "West"},
            1: {"name": "Marker 1 (Row 6, Col 6 / Top-Right)", "expected_deg": 180.0, "expected_dir": "West"},
            2: {"name": "Marker 2 (Row 11, Col 0 / Bottom-Left)", "expected_deg": 0.0, "expected_dir": "East"},
            3: {"name": "Marker 3 (Row 11, Col 6 / Bottom-Right)", "expected_deg": 0.0, "expected_dir": "East"},
        }

        corners, ids = self.detect_aruco_markers(frame)
        h, w = frame.shape[:2]

        reports = {}
        all_detected = True
        all_aligned = True

        detected_map = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                detected_map[int(mid)] = corners[i][0]

        # Calculate temporary H matrix to map orientation into arena coordinate system
        temp_H = None
        if all(k in detected_map for k in [0, 1, 3, 2]):
            src_pts = np.array([
                np.mean(detected_map[0], axis=0),
                np.mean(detected_map[1], axis=0),
                np.mean(detected_map[3], axis=0),
                np.mean(detected_map[2], axis=0),
            ], dtype=np.float32)
            temp_H = cv2.getPerspectiveTransform(src_pts, self.dst_pts)

        for mid, spec in specs.items():
            if mid not in detected_map:
                all_detected = False
                all_aligned = False
                reports[mid] = {
                    "id": mid,
                    "name": spec["name"],
                    "detected": False,
                    "is_oriented_correctly": False,
                    "orientation_error_deg": 999.0,
                    "measured_angle_deg": None,
                    "edge_clearance_px": 0.0,
                    "advice": f"{spec['name']} is NOT detected. Check if occluded or partially outside camera frame.",
                }
                continue

            c = detected_map[mid]
            center = np.mean(c, axis=0)
            min_dist_to_edge = min(center[0], w - center[0], center[1], h - center[1])

            # Top edge heading vector (c1 - c0)
            v = c[1] - c[0]
            
            # Map into physical arena frame if homography is available
            if temp_H is not None:
                p0 = np.array([center[0], center[1], 1.0])
                p1 = np.array([center[0] + v[0], center[1] + v[1], 1.0])
                P0 = temp_H @ p0
                P0 = P0[:2] / P0[2]
                P1 = temp_H @ p1
                P1 = P1[:2] / P1[2]
                V_arena = P1 - P0
                arena_angle = float(np.degrees(np.arctan2(V_arena[1], V_arena[0])))
            else:
                arena_angle = float(np.degrees(np.arctan2(v[1], v[0])))

            # Angle error relative to expected heading in arena frame
            ang_err = (arena_angle - spec["expected_deg"] + 180.0) % 360.0 - 180.0
            is_oriented = abs(ang_err) <= 25.0

            if not is_oriented:
                all_aligned = False
                rot_dir = "counter-clockwise" if ang_err > 0 else "clockwise"
                advice = (
                    f"{spec['name']} is tilted by {abs(ang_err):.1f}° (points at {arena_angle:.1f}°, "
                    f"expected {spec['expected_dir']} {spec['expected_deg']}°). "
                    f"Please rotate marker {abs(ang_err):.0f}° {rot_dir}."
                )
            elif min_dist_to_edge < 25.0:
                advice = (
                    f"{spec['name']} is oriented correctly, but sits close to the camera frame boundary "
                    f"({min_dist_to_edge:.0f}px from edge). Consider shifting arena slightly inward."
                )
            else:
                advice = f"{spec['name']} is correctly placed and oriented ({spec['expected_dir']})."

            reports[mid] = {
                "id": mid,
                "name": spec["name"],
                "detected": True,
                "is_oriented_correctly": is_oriented,
                "orientation_error_deg": round(abs(ang_err), 1),
                "measured_angle_deg": round(arena_angle, 1),
                "edge_clearance_px": round(float(min_dist_to_edge), 1),
                "advice": advice,
            }

        return {
            "all_detected": all_detected,
            "all_aligned": all_aligned,
            "reports": reports,
        }

    def lock_homography(self, locked: bool = True):
        """Freezes or unlocks the homography matrix to prevent drift or jitter."""
        self.H_locked = locked

    def warp_arena(self, frame: np.ndarray) -> Optional[np.ndarray]:
        """Warps the full camera image into the rectified metric arena grid (600x500 px)."""
        if not self.H_locked:
            self.calibrate_homography(frame, force_update=True)
        if self.H_matrix is None:
            return None
        return cv2.warpPerspective(frame, self.H_matrix, (self.rect_w, self.rect_h))

    def pixel_to_world(self, u: float, v: float) -> Tuple[float, float, float]:
        """
        Maps rectified arena image pixel (u, v) to physical world millimeter coordinates (Xw, Yw, Zw).
        u=0 is Left (X=25.0 mm), u=600 is Right (X=325.0 mm).
        v=0 is Top (Row 6, Y=275.0 mm), v=500 is Bottom (Row 11, Y=25.0 mm).
        """
        x_min = 25.0
        y_max = 275.0
        x_w = x_min + (u / self.scale_px_per_mm)
        y_w = y_max - (v / self.scale_px_per_mm)
        z_w = self.cube_center_z_mm  # Tabletop surface Z=0.0 mm + half cube/block height
        return (round(x_w, 2), round(y_w, 2), z_w)

    def world_to_pixel(self, x_w: float, y_w: float) -> Tuple[int, int]:
        """Maps physical world millimeters (Xw, Yw) back to rectified pixel (u, v)."""
        x_min = 25.0
        y_max = 275.0
        u = int(round((x_w - x_min) * self.scale_px_per_mm))
        v = int(round((y_max - y_w) * self.scale_px_per_mm))
        return (u, v)

    def world_to_arm_polar(self, x_w: float, y_w: float, arm_id: str = "xarm_1") -> Tuple[float, float]:
        """
        Transforms global world coordinates (Xw, Yw) into arm-relative cylindrical/polar coordinates (r, theta_arm).
        Single arm base is at (200.0, 280.0) with nominal heading -90.0° (pointing South toward -Y).
        """
        arm_info = self.base_mounts.get(arm_id, self.base_mounts["xarm_1"])
        bx = arm_info["x"]
        by = arm_info["y"]
        heading = arm_info["heading"]

        dx = x_w - bx
        dy = y_w - by
        r = math.hypot(dx, dy)
        theta_world = math.degrees(math.atan2(dy, dx))
        
        # Relative angle from arm nominal heading
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
            if not color_info.get("enabled", True):
                continue

            bands = color_info.get("bands", [])
            if not bands:
                continue

            targetable = color_info.get("targetable", True)

            combined_mask = np.zeros(warped_frame.shape[:2], dtype=np.uint8)
            for band in bands:
                lower = np.array([band["h_min"], band["s_min"], band["v_min"]], dtype=np.uint8)
                upper = np.array([band["h_max"], band["s_max"], band["v_max"]], dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                combined_mask = cv2.bitwise_or(combined_mask, mask)

            if open_iters > 0:
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=open_iters)
            if close_iters > 0:
                combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=close_iters)

            # Robot Chassis Exclusion: Mask out Rows 3..5, Cols 2..4 (X in [100, 250], Y in [300, 450]) if within arena
            u_b1, v_b1 = self.world_to_pixel(100.0, 450.0)
            u_b2, v_b2 = self.world_to_pixel(250.0, 300.0)
            u_min = max(0, min(u_b1, u_b2))
            u_max = max(0, min(warped_frame.shape[1], max(u_b1, u_b2)))
            v_min = max(0, min(v_b1, v_b2))
            v_max = max(0, min(warped_frame.shape[0], max(v_b1, v_b2)))
            if u_max > u_min and v_max > v_min:
                combined_mask[v_min:v_max, u_min:u_max] = 0

            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (min_area <= area <= max_area):
                    continue

                rect = cv2.minAreaRect(cnt)
                (cx, cy), (w_box, h_box), angle = rect

                aspect = max(w_box, h_box) / max(min(w_box, h_box), 1e-4)
                if aspect > max_aspect:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    u_c = M["m10"] / M["m00"]
                    v_c = M["m01"] / M["m00"]
                else:
                    u_c, v_c = cx, cy

                yaw = angle - 90.0 if w_box < h_box else angle
                while yaw < -45.0:
                    yaw += 90.0
                while yaw > 45.0:
                    yaw -= 90.0

                # Analyze rectangular block orientation & wrist roll (Motor 2) alignment
                box_pts = cv2.boxPoints(rect)
                p0, p1, p2, p3 = box_pts[0], box_pts[1], box_pts[2], box_pts[3]
                d01 = float(np.linalg.norm(p1 - p0))
                d12 = float(np.linalg.norm(p2 - p1))
                if d01 >= d12:
                    long_vec = p1 - p0
                else:
                    long_vec = p2 - p1

                long_dx = abs(float(long_vec[0]))
                long_dy = abs(float(long_vec[1]))

                # In rectified image: u (x-axis) is along World X, v (y-axis) is along World -Y.
                # Motor 2 at neutral 500 ticks has claw jaws level (opening horizontally along World X, span 55mm).
                # If block is elongated (aspect >= 1.25, e.g. 60x35mm craft blocks):
                #   If long_dx > long_dy: longer dimension (60mm) is along World X.
                #     The 55mm claw span cannot span 60mm; rotate Motor 2 CW by 90° (130 ticks)
                #     so jaws open along World Y to clamp the narrow 35mm width safely!
                #   Else: longer dimension is along World Y.
                #     Narrow dimension (35mm) is already along World X; keep neutral roll at 500 ticks.
                # If block is square (aspect < 1.25): compute face-aligned roll after polar transform.
                if aspect >= 1.25:
                    if long_dx > long_dy:
                        gripper_roll_tick = 130
                    else:
                        gripper_roll_tick = 500
                else:
                    gripper_roll_tick = 500

                # Oblique camera elevation correction for tall/glued cubes
                bx, by, bw, bh = cv2.boundingRect(cnt)
                if bh > bw * 1.3:
                    v_base = by + bh
                    y_w_base = 275.0 - (v_base / self.scale_px_per_mm)
                    y_w = y_w_base + 12.5  # Half cube footprint offset
                    u_c = bx + bw / 2.0
                    x_w = 25.0 + (u_c / self.scale_px_per_mm)
                    z_w = 19.05  # Doubled block center
                else:
                    x_w, y_w, _ = self.pixel_to_world(u_c, v_c)
                    z_w = self.cube_center_z_mm   # Dynamic cube center Z from xarm_hardware.yaml

                # 3D Parallax Correction using 5.0cm marker PnP extrinsics & 3.75x3.75x4.5cm geometry:
                # 1) Oblique camera mount (depression < 70°): project elevated centroid back onto table footprint.
                # 2) Overhead camera (depression >= 70°): elevated 4.5cm top face + visible side walls shift
                #    the apparent centroid radially outward from (cam_x, cam_y) by (r_cam * z_eff / H_c).
                if self.camera_extrinsics and self.camera_extrinsics.get("success"):
                    dep_angle = float(self.camera_extrinsics.get("depression_angle_deg", 90.0))
                    side = str(self.camera_extrinsics.get("viewing_side", ""))
                    cam_pos = self.camera_extrinsics.get("camera_position_world_mm", {})
                    cam_x = float(cam_pos.get("x", 175.0))
                    cam_y = float(cam_pos.get("y", 150.0))
                    cam_h = float(cam_pos.get("z", 0.0))
                    if dep_angle < 70.0:
                        rad = math.radians(max(20.0, dep_angle))
                        shift_mm = z_w / math.tan(rad)
                        if "East" in side:
                            x_w -= shift_mm
                        elif "West" in side:
                            x_w += shift_mm
                        elif "South" in side:
                            y_w += shift_mm
                        elif "North" in side:
                            y_w -= shift_mm
                    elif cam_h > 150.0:
                        # Near straight-down angles (dep_angle >= 80°), planar solvePnP lateral (cam_x, cam_y)
                        # is noisy; clamp within the active arena so parallax cannot introduce lateral skew.
                        if dep_angle >= 80.0:
                            cam_x = max(25.0, min(325.0, cam_x))
                            cam_y = max(25.0, min(275.0, cam_y))
                        # Blend apparent elevation from cube center Z (22.5mm) toward top face (36mm) when
                        # visible side walls enlarge apparent footprint beyond top face area (37.5^2 = 1406mm^2).
                        top_area_mm2 = self.cube_size_mm * self.cube_size_mm
                        apparent_mm2 = float(area) / (self.scale_px_per_mm * self.scale_px_per_mm)
                        excess_frac = max(0.0, min(1.0, (apparent_mm2 - top_area_mm2) / max(top_area_mm2 * 0.6, 1.0)))
                        z_eff = z_w + excess_frac * (0.8 * self.cube_height_mm - z_w)
                        x_w -= (x_w - cam_x) * (z_eff / cam_h)
                        y_w -= (y_w - cam_y) * (z_eff / cam_h)

                # Clamp within physical arena bounds
                x_w = max(25.0, min(325.0, x_w))
                y_w = max(25.0, min(275.0, y_w))

                # Exclude Rows 3..5: objects must be within active picking Rows 6..10
                if y_w > 280.0 or y_w < 20.0:
                    continue

                r, th = self.world_to_arm_polar(x_w, y_w, "xarm_1")

                # For near-field square cubes (r < 195mm, top-down grip), align Motor 2 (wrist_roll)
                # to the nearest flat cube face relative to arm heading (th).
                # For far square cubes (r >= 195mm, cup-style angled grip), keep Motor 2 level at 500
                # so wrist roll doesn't tilt one jaw downward into the table.
                if aspect < 1.25:
                    if r < 195.0:
                        rel_ccw_deg = ((-yaw) - th + 45.0) % 90.0 - 45.0
                        # On mounted xArm Motor 2, increasing ticks (>500) rotates CW from overhead view
                        gripper_roll_tick = max(130, min(870, int(round(500.0 - rel_ccw_deg * (740.0 / 180.0)))))
                    else:
                        gripper_roll_tick = 500

                # Reach envelope check (must be reachable by arm: 110 mm <= r <= 325 mm)
                if 110.0 <= r <= 325.0:
                    box_points = box_pts.astype(np.int32)
                    cube = DetectedCube(
                        color=color_key,
                        world_x=round(x_w, 2),
                        world_y=round(y_w, 2),
                        world_z=z_w,
                        yaw_deg=round(yaw, 2),
                        assigned_arm="xarm_1",
                        r_mm=r,
                        theta_deg=th,
                        target_r_mm=r,
                        target_theta_deg=th,
                        r1_mm=r,
                        theta1_deg=th,
                        r2_mm=r,
                        theta2_deg=th,
                        requires_mutex=False,
                        pixel_center=(round(u_c, 1), round(v_c, 1)),
                        bounding_box=box_points,
                        contour_area_px=float(area),
                        targetable=targetable,
                        angle_deg=round(angle, 2),
                        width_px=round(float(min(w_box, h_box)), 1),
                        height_px=round(float(max(w_box, h_box)), 1),
                        gripper_roll_tick=gripper_roll_tick,
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

        if draw_grid:
            # Vertical lines (every 50 mm)
            x_min = int(self.marker_world_corners[0][0])
            x_max = int(self.marker_world_corners[1][0])
            for x_mm in range(((x_min // 50) + 1) * 50, x_max, 50):
                u, _ = self.world_to_pixel(float(x_mm), (self.marker_world_corners[0][1] + self.marker_world_corners[2][1]) / 2.0)
                if 0 <= u < self.rect_w:
                    cv2.line(annotated, (u, 0), (u, self.rect_h), (60, 60, 60), 1)
                    cv2.putText(
                        annotated, f"{x_mm}", (u + 2, self.rect_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA
                    )

            # Horizontal lines (every 50 mm)
            y_min = int(self.marker_world_corners[2][1])
            y_max = int(self.marker_world_corners[0][1])
            for y_mm in range(((y_min // 50) + 1) * 50, y_max, 50):
                _, v = self.world_to_pixel(self.midline_x_mm, float(y_mm))
                if 0 <= v < self.rect_h:
                    cv2.line(annotated, (0, v), (self.rect_w, v), (60, 60, 60), 1)
                    cv2.putText(
                        annotated, f"{y_mm}", (4, v - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA
                    )

            # Centerline (X = 175 mm) in cyan
            mid_u, _ = self.world_to_pixel(self.midline_x_mm, 100.0)
            if 0 <= mid_u < self.rect_w:
                cv2.line(annotated, (mid_u, 0), (mid_u, self.rect_h), (255, 255, 0), 1)
                cv2.putText(
                    annotated, "MIDLINE X=175", (mid_u - 45, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA
                )

            # Highlight Golden Pickup Zone (Rows 7..9, Cols 2..4: X in [75, 275], Y in [100, 250])
            u_g1, v_g1 = self.world_to_pixel(75.0, 250.0)
            u_g2, v_g2 = self.world_to_pixel(275.0, 100.0)
            zone_overlay = annotated.copy()
            cv2.rectangle(zone_overlay, (u_g1, v_g1), (u_g2, v_g2), (40, 180, 40), -1)
            cv2.addWeighted(zone_overlay, 0.20, annotated, 0.80, 0, annotated)
            cv2.rectangle(annotated, (u_g1, v_g1), (u_g2, v_g2), (0, 220, 0), 2)
            cv2.putText(
                annotated, "GOLDEN PICKUP ZONE (Rows 7-9, Cols 2-4)", (u_g1 + 8, v_g1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 0), 1, cv2.LINE_AA
            )

            # Label ArUco Markers matching user nomenclature & physical IDs
            marker_labels = {
                (25.0, 275.0): ("M0 (ID 2)", 8, 35),
                (325.0, 275.0): ("M1 (ID 0)", -85, 35),
                (25.0, 25.0): ("M2 (ID 3)", 8, -15),
                (325.0, 25.0): ("M3 (ID 1)", -85, -15),
            }
            for (mx, my), (mtext, off_x, off_y) in marker_labels.items():
                mu, mv = self.world_to_pixel(mx, my)
                cv2.putText(
                    annotated, mtext, (mu + off_x, mv + off_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA
                )

        for cube in cubes:
            color_bgr = self.color_config.get("colors", {}).get(cube.color, {}).get("bgr_display", [0, 255, 255])
            
            if cube.bounding_box is not None:
                cv2.drawContours(annotated, [cube.bounding_box], 0, color_bgr, 2)

            uc, vc = int(round(cube.pixel_center[0])), int(round(cube.pixel_center[1]))
            cv2.circle(annotated, (uc, vc), 4, (0, 0, 0), -1)
            cv2.circle(annotated, (uc, vc), 3, (255, 255, 255), -1)

            arrow_len = 16.0
            rad = math.radians(cube.yaw_deg)
            ex = int(round(uc + arrow_len * math.cos(rad)))
            ey = int(round(vc - arrow_len * math.sin(rad)))
            cv2.arrowedLine(annotated, (uc, vc), (ex, ey), (0, 0, 255), 2, tipLength=0.3)

            tag = f"{cube.color.upper()}"
            roll_str = f" roll:{cube.gripper_roll_tick}" if cube.gripper_roll_tick != 500 else ""
            coords = f"r:{cube.target_r_mm:.0f}mm th:{cube.target_theta_deg:+.0f}°{roll_str}"
            
            text_y = max(vc - 16, 20)
            cv2.putText(annotated, tag, (uc - 20, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, tag, (uc - 20, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(annotated, coords, (uc - 35, text_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(annotated, coords, (uc - 35, text_y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.33, (220, 220, 220), 1, cv2.LINE_AA)

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
