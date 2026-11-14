#!/usr/bin/env python3
"""
Interactive HSV Color Segmentation Tuning Tool.

Features:
  - Real-time 6-trackbar GUI (H_min, H_max, S_min, S_max, V_min, V_max).
  - Switch active color on-the-fly: [1] Red, [2] Green, [3] Blue, [4] Yellow.
  - Automatically handles Red dual-hue wrap ([0, 10] and [170, 180]).
  - Displays live side-by-side feed: Rectified Arena | Binary Color Mask | Detection Contours.
  - Press 's' to atomically export and save tuned thresholds directly to config/colors.yaml.
  - Supports --synthetic mode to test offline without physical camera.
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
from vision.test_aruco_warp import generate_synthetic_arena_frame
import yaml


def nothing(x):
    pass


class HSVTuner:
    def __init__(self, config_path: str = "config/colors.yaml", pipeline: Optional[ArenaVisionPipeline] = None):
        self.config_path = config_path
        self.pipeline = pipeline or ArenaVisionPipeline()
        self.active_color = "red"
        self.color_keys = ["red", "green", "blue", "yellow"]
        self.window_name = "HSV Threshold Tuner"

        # Load existing config
        self.load_config()

        # Cache trackbar values per color
        self.thresholds = {}
        for c in self.color_keys:
            c_info = self.config_data.get("colors", {}).get(c, {})
            bands = c_info.get("bands", [])
            if bands:
                b = bands[0]
                self.thresholds[c] = {
                    "h_min": b.get("h_min", 0),
                    "h_max": b.get("h_max", 180),
                    "s_min": b.get("s_min", 100),
                    "s_max": b.get("s_max", 255),
                    "v_min": b.get("v_min", 50),
                    "v_max": b.get("v_max", 255),
                }
            else:
                self.thresholds[c] = {"h_min": 0, "h_max": 180, "s_min": 100, "s_max": 255, "v_min": 50, "v_max": 255}

    def load_config(self):
        if os.path.exists(self.config_path):
            with open(self.config_path, "r") as f:
                self.config_data = yaml.safe_load(f) or {}
        else:
            self.config_data = {}

    def setup_trackbars(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("H_min", self.window_name, 0, 180, nothing)
        cv2.createTrackbar("H_max", self.window_name, 180, 180, nothing)
        cv2.createTrackbar("S_min", self.window_name, 0, 255, nothing)
        cv2.createTrackbar("S_max", self.window_name, 255, 255, nothing)
        cv2.createTrackbar("V_min", self.window_name, 0, 255, nothing)
        cv2.createTrackbar("V_max", self.window_name, 255, 255, nothing)

        self.update_trackbars_for_color(self.active_color)

    def update_trackbars_for_color(self, color: str):
        self.active_color = color
        t = self.thresholds[color]
        cv2.setTrackbarPos("H_min", self.window_name, t["h_min"])
        cv2.setTrackbarPos("H_max", self.window_name, t["h_max"])
        cv2.setTrackbarPos("S_min", self.window_name, t["s_min"])
        cv2.setTrackbarPos("S_max", self.window_name, t["s_max"])
        cv2.setTrackbarPos("V_min", self.window_name, t["v_min"])
        cv2.setTrackbarPos("V_max", self.window_name, t["v_max"])

    def read_trackbars(self):
        t = self.thresholds[self.active_color]
        t["h_min"] = cv2.getTrackbarPos("H_min", self.window_name)
        t["h_max"] = cv2.getTrackbarPos("H_max", self.window_name)
        t["s_min"] = cv2.getTrackbarPos("S_min", self.window_name)
        t["s_max"] = cv2.getTrackbarPos("S_max", self.window_name)
        t["v_min"] = cv2.getTrackbarPos("V_min", self.window_name)
        t["v_max"] = cv2.getTrackbarPos("V_max", self.window_name)
        return t

    def save_config(self):
        """Atomically saves calibrated thresholds to config/colors.yaml."""
        if "colors" not in self.config_data:
            self.config_data["colors"] = {}

        for c in self.color_keys:
            t = self.thresholds[c]
            if c not in self.config_data["colors"]:
                self.config_data["colors"][c] = {"name": c.capitalize()}

            if c == "red":
                # Red uses dual bands: [0, h_max] and [180 - h_max, 180]
                h_span = min(max(t["h_max"], 5), 25)
                self.config_data["colors"][c]["bands"] = [
                    {"h_min": 0, "h_max": h_span, "s_min": t["s_min"], "s_max": t["s_max"], "v_min": t["v_min"], "v_max": t["v_max"]},
                    {"h_min": 180 - h_span, "h_max": 180, "s_min": t["s_min"], "s_max": t["s_max"], "v_min": t["v_min"], "v_max": t["v_max"]},
                ]
            else:
                self.config_data["colors"][c]["bands"] = [
                    {"h_min": t["h_min"], "h_max": t["h_max"], "s_min": t["s_min"], "s_max": t["s_max"], "v_min": t["v_min"], "v_max": t["v_max"]}
                ]

        temp_path = self.config_path + ".tmp"
        with open(temp_path, "w") as f:
            yaml.dump(self.config_data, f, sort_keys=False, default_flow_style=False)
        os.replace(temp_path, self.config_path)
        print(f"[Success] Saved tuned color configurations to {self.config_path}!")


def main():
    parser = argparse.ArgumentParser(description="Interactive HSV Color Tuner.")
    parser.add_argument("--device", default="/dev/video0", help="Camera device path or index (default: /dev/video0)")
    parser.add_argument("--config", default="config/colors.yaml", help="Path to colors.yaml")
    parser.add_argument("--synthetic", action="store_true", help="Run offline tuner with synthetic arena frame")
    args = parser.parse_args()

    pipeline = ArenaVisionPipeline(config_dir=os.path.dirname(args.config))
    tuner = HSVTuner(config_path=args.config, pipeline=pipeline)
    tuner.setup_trackbars()

    print("========================================================================")
    print("  INTERACTIVE HSV COLOR THRESHOLD TUNER")
    print("  Active Keys:")
    print("    [1] Select RED    [2] Select GREEN    [3] Select BLUE    [4] Select YELLOW")
    print(f"    [s] Save current thresholds to {args.config}")
    print("    [q] Quit tuner")
    print("========================================================================")

    if args.synthetic:
        frame = generate_synthetic_arena_frame()
        pipeline.calibrate_homography(frame)
        warped = pipeline.warp_arena(frame)
        if warped is None:
            print("[Error] Could not warp synthetic frame.")
            sys.exit(1)

        while True:
            t = tuner.read_trackbars()
            hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

            if tuner.active_color == "red":
                h_span = min(max(t["h_max"], 5), 25)
                m1 = cv2.inRange(hsv, np.array([0, t["s_min"], t["v_min"]]), np.array([h_span, t["s_max"], t["v_max"]]))
                m2 = cv2.inRange(hsv, np.array([180 - h_span, t["s_min"], t["v_min"]]), np.array([180, t["s_max"], t["v_max"]]))
                mask = cv2.bitwise_or(m1, m2)
            else:
                mask = cv2.inRange(hsv, np.array([t["h_min"], t["s_min"], t["v_min"]]), np.array([t["h_max"], t["s_max"], t["v_max"]]))

            mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, pipeline.morph_kernel)
            mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, pipeline.morph_kernel)

            # Segmented preview
            segmented = cv2.bitwise_and(warped, warped, mask=mask_clean)
            mask_bgr = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)

            combined_preview = np.hstack([warped, mask_bgr, segmented])
            status_text = f"Active: {tuner.active_color.upper()} (Keys 1:Red, 2:Grn, 3:Blu, 4:Yel | 's' Save | 'q' Quit)"
            cv2.putText(combined_preview, status_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow(tuner.window_name, combined_preview)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('1'):
                tuner.update_trackbars_for_color("red")
            elif key == ord('2'):
                tuner.update_trackbars_for_color("green")
            elif key == ord('3'):
                tuner.update_trackbars_for_color("blue")
            elif key == ord('4'):
                tuner.update_trackbars_for_color("yellow")
            elif key == ord('s'):
                tuner.save_config()

        cv2.destroyAllWindows()
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

        warped = pipeline.warp_arena(pipeline.undistort(frame))
        if warped is None:
            # Show unwarped frame if markers not yet locked
            cv2.putText(frame, "Waiting for 4 ArUco markers...", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow(tuner.window_name, cv2.resize(frame, (960, 540)))
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
            continue

        t = tuner.read_trackbars()
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        if tuner.active_color == "red":
            h_span = min(max(t["h_max"], 5), 25)
            m1 = cv2.inRange(hsv, np.array([0, t["s_min"], t["v_min"]]), np.array([h_span, t["s_max"], t["v_max"]]))
            m2 = cv2.inRange(hsv, np.array([180 - h_span, t["s_min"], t["v_min"]]), np.array([180, t["s_max"], t["v_max"]]))
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(hsv, np.array([t["h_min"], t["s_min"], t["v_min"]]), np.array([t["h_max"], t["s_max"], t["v_max"]]))

        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, pipeline.morph_kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, pipeline.morph_kernel)

        segmented = cv2.bitwise_and(warped, warped, mask=mask_clean)
        mask_bgr = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)
        combined_preview = np.hstack([warped, mask_bgr, segmented])

        status_text = f"Active: {tuner.active_color.upper()} (Keys 1:Red, 2:Grn, 3:Blu, 4:Yel | 's' Save | 'q' Quit)"
        cv2.putText(combined_preview, status_text, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.imshow(tuner.window_name, combined_preview)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('1'):
            tuner.update_trackbars_for_color("red")
        elif key == ord('2'):
            tuner.update_trackbars_for_color("green")
        elif key == ord('3'):
            tuner.update_trackbars_for_color("blue")
        elif key == ord('4'):
            tuner.update_trackbars_for_color("yellow")
        elif key == ord('s'):
            tuner.save_config()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
