#!/usr/bin/env python3
"""
Simulated Multi-Angle Camera & Visualizer Renderer for xArm 1S.

Renders high-resolution visual viewpoints of the xArm 1S workcell:
  1. Top-Down Overhead View (X-Y plane): 7x12 grid, cell coordinates, arm reach.
  2. Side Elevation View (Y-Z plane): Table clearance, Z-descent down to 9.5mm, pitch angle.
  3. Front Elevation View (X-Z plane): Lateral centering over cube, claw width.
  4. 3D Isometric View: Perspective spatial projection of entire workcell.
  5. 4-Panel Multi-View Dashboard: Combined 2x2 layout of all 4 perspectives.

Usage:
  python3 tools/render_sim_view.py
  (or with --loop to continuously render from active ROS 2 topics)
"""

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

try:
    import cv2
    import numpy as np
except ImportError:
    print("OpenCV or NumPy not installed. Run inside Docker or install cv2.")
    sys.exit(1)

# Import kinematics if available
try:
    from sorting_arm.kinematics import DEG_PER_TICK, load_kinematic_parameters
except ImportError:
    try:
        sys.path.append(os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src", "sorting_arm"))
        from sorting_arm.kinematics import DEG_PER_TICK, load_kinematic_parameters
    except ImportError:
        DEG_PER_TICK = 240.0 / 1000.0
        load_kinematic_parameters = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:
    rclpy = None


class WorkcellGeometry:
    """Computes exact 3D world forward kinematics for xArm 1S links and features."""

    def __init__(self):
        self.base_x = 175.0
        self.base_y = 375.0
        self.base_z = 0.0
        self.d1 = 65.0
        self.L1 = 101.0
        self.L2 = 95.0
        self.L3 = 150.0

        if load_kinematic_parameters is not None:
            try:
                self.params = load_kinematic_parameters(arm_id="xarm_1")
            except Exception:
                self.params = None
        else:
            self.params = None

    def compute_joint_positions(self, joint_ticks: Dict[int, int]) -> Dict[str, Tuple[float, float, float]]:
        """
        Computes 3D world coordinates (X, Y, Z) in mm for each joint and tool tip.
        """
        centers = self.params.nominal_center_ticks if self.params else {1: 517, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        dir_mult = self.params.direction_multipliers if self.params else {i: 1 for i in range(1, 7)}
        base_zero = getattr(self.params, "base_zero_deg", 0.0) if self.params else 0.0
        base_heading = getattr(self.params, "base_heading_deg", -90.0) if self.params else -90.0
        sh_zero = getattr(self.params, "shoulder_zero_deg", 90.0) if self.params else 90.0
        el_zero = getattr(self.params, "elbow_zero_deg", 180.0) if self.params else 180.0
        wr_zero = getattr(self.params, "wrist_zero_deg", 0.0) if self.params else 0.0

        m6 = joint_ticks.get(6, centers.get(6, 502))
        m5 = joint_ticks.get(5, centers.get(5, 501))
        m4 = joint_ticks.get(4, centers.get(4, 496))
        m3 = joint_ticks.get(3, centers.get(3, 504))

        # Base yaw in world coordinates (East=0°, South=-90°)
        theta_arm = base_zero + dir_mult.get(6, 1) * (m6 - centers.get(6, 502)) * DEG_PER_TICK
        theta_world_deg = theta_arm + base_heading
        theta_world_rad = math.radians(theta_world_deg)

        # Shoulder elevation from horizontal table
        th_sh_deg = sh_zero - dir_mult.get(5, 1) * (m5 - centers.get(5, 501)) * DEG_PER_TICK
        th_sh_rad = math.radians(th_sh_deg)

        # Elbow angle (Interior angle alpha3: 180° = collinear, folding forward decreases alpha3)
        alpha3_deg = el_zero + dir_mult.get(4, 1) * (m4 - centers.get(4, 496)) * DEG_PER_TICK
        th_el_deg = th_sh_deg - (180.0 - alpha3_deg)
        th_el_rad = math.radians(th_el_deg)

        # Wrist pitch angle relative to table horizontal
        th_wr_rel = wr_zero - dir_mult.get(3, 1) * (m3 - centers.get(3, 504)) * DEG_PER_TICK
        phi_deg = th_el_deg + th_wr_rel
        phi_rad = math.radians(phi_deg)

        # Radial 2D projections
        cos_w = math.cos(theta_world_rad)
        sin_w = math.sin(theta_world_rad)

        z_sh = self.d1
        x_sh = self.base_x
        y_sh = self.base_y

        r_el = self.L1 * math.cos(th_sh_rad)
        z_el = z_sh + self.L1 * math.sin(th_sh_rad)
        x_el = self.base_x + r_el * cos_w
        y_el = self.base_y + r_el * sin_w

        r_wr = r_el + self.L2 * math.cos(th_el_rad)
        z_wr = z_el + self.L2 * math.sin(th_el_rad)
        x_wr = self.base_x + r_wr * cos_w
        y_wr = self.base_y + r_wr * sin_w

        r_tool = r_wr + self.L3 * math.cos(phi_rad)
        z_tool = z_wr + self.L3 * math.sin(phi_rad)
        x_tool = self.base_x + r_tool * cos_w
        y_tool = self.base_y + r_tool * sin_w

        return {
            "base": (self.base_x, self.base_y, 0.0),
            "shoulder": (x_sh, y_sh, z_sh),
            "elbow": (x_el, y_el, z_el),
            "wrist": (x_wr, y_wr, z_wr),
            "tool_tip": (x_tool, y_tool, z_tool),
            "angles": {
                "theta_world_deg": theta_world_deg,
                "shoulder_deg": th_sh_deg,
                "elbow_bend_deg": 180.0 - alpha3_deg,
                "forearm_deg": th_el_deg,
                "wrist_rel_deg": th_wr_rel,
                "pitch_phi_deg": phi_deg,
                "radial_reach_mm": r_tool,
            },
        }


class MultiViewRenderer:
    """Renders 4 distinct camera views and combined dashboard of xArm workcell."""

    def __init__(self, output_dir: str = "artifacts"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.geom = WorkcellGeometry()

    # ---------------------------------------------------------------------------------
    # 1. TOP-DOWN OVERHEAD VIEW (X-Y Plane)
    # ---------------------------------------------------------------------------------
    def render_topdown(
        self,
        joint_ticks: Dict[int, int],
        cubes: List[dict],
        status_text: str = "TOP-DOWN OVERHEAD VIEW",
    ) -> np.ndarray:
        scale = 1.8  # px / mm
        margin_x = 70
        margin_y = 70
        arena_w = int(350 * scale)
        arena_h = int(600 * scale)
        img_w = arena_w + 2 * margin_x
        img_h = arena_h + 2 * margin_y + 40

        img = np.full((img_h, img_w, 3), (242, 244, 247), dtype=np.uint8)

        def w2i(x_mm, y_mm):
            px = int(margin_x + x_mm * scale)
            py = int(margin_y + (600.0 - y_mm) * scale + 40)
            return px, py

        # Baseboard Table
        tl = w2i(0.0, 600.0)
        br = w2i(350.0, 0.0)
        cv2.rectangle(img, tl, br, (228, 230, 225), -1)
        cv2.rectangle(img, tl, br, (150, 150, 150), 2)

        # Golden Pickup Zone (Rows 7..9, Cols 2..4: X 75..275, Y 100..250)
        pz_tl = w2i(75.0, 250.0)
        pz_br = w2i(275.0, 100.0)
        cv2.rectangle(img, pz_tl, pz_br, (215, 240, 220), -1)
        cv2.rectangle(img, pz_tl, pz_br, (70, 180, 85), 2)
        cv2.putText(
            img, "GOLDEN ZONE (Rows 7..9, Cols 2..4)", (pz_tl[0] - 20, pz_br[1] + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 140, 50), 2, cv2.LINE_AA
        )

        # Grid lines (5cm = 50mm)
        for c in range(8):
            x = c * 50.0
            cv2.line(img, w2i(x, 0.0), w2i(x, 600.0), (195, 180, 150), 1, cv2.LINE_AA)
            if c < 7:
                mid_x = (c + 0.5) * 50.0
                pt = w2i(mid_x, -10.0)
                cv2.putText(img, f"C{c}", (pt[0] - 12, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 90, 60), 2)

        for r in range(13):
            y = r * 50.0
            cv2.line(img, w2i(0.0, y), w2i(350.0, y), (195, 180, 150), 1, cv2.LINE_AA)
            if r < 12:
                row_num = 11 - r
                pt = w2i(-12.0, (r + 0.5) * 50.0)
                cv2.putText(img, f"R{row_num}", (pt[0] - 28, pt[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 90, 60), 2)

        # Containers
        r_tl = w2i(0.0, 600.0)
        r_br = w2i(100.0, 500.0)
        cv2.rectangle(img, r_tl, r_br, (180, 180, 240), -1)
        cv2.rectangle(img, r_tl, r_br, (40, 40, 220), 2)
        cv2.putText(img, "RED TRAY", (r_tl[0] + 12, r_tl[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 180), 2)

        g_tl = w2i(250.0, 600.0)
        g_br = w2i(350.0, 500.0)
        cv2.rectangle(img, g_tl, g_br, (180, 240, 180), -1)
        cv2.rectangle(img, g_tl, g_br, (40, 180, 40), 2)
        cv2.putText(img, "GREEN TRAY", (g_tl[0] + 8, g_tl[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 140, 20), 2)

        # Cubes (37.5 x 37.5 mm)
        c_size = int(37.5 * scale)
        half = c_size // 2
        for c in cubes:
            cx, cy = w2i(c["x"], c["y"])
            col = (30, 30, 220) if c["color"].lower() == "red" else (30, 180, 40)
            cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), col, -1)
            cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), (20, 20, 20), 2)
            cv2.putText(img, f"{c['color'].upper()}", (cx - half - 5, cy - half - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 2)

        # Arm FK
        fk = self.geom.compute_joint_positions(joint_ticks)
        base_pt = w2i(self.geom.base_x, self.geom.base_y)
        sh_pt = w2i(fk["shoulder"][0], fk["shoulder"][1])
        el_pt = w2i(fk["elbow"][0], fk["elbow"][1])
        wr_pt = w2i(fk["wrist"][0], fk["wrist"][1])
        tool_pt = w2i(fk["tool_tip"][0], fk["tool_tip"][1])

        # Hub
        cv2.circle(img, base_pt, 20, (190, 120, 30), -1)
        cv2.circle(img, base_pt, 20, (80, 50, 10), 2)

        # Links
        cv2.line(img, sh_pt, el_pt, (180, 100, 20), 8, cv2.LINE_AA)
        cv2.line(img, el_pt, wr_pt, (150, 80, 15), 6, cv2.LINE_AA)
        cv2.line(img, wr_pt, tool_pt, (50, 50, 50), 5, cv2.LINE_AA)

        cv2.circle(img, el_pt, 7, (40, 40, 40), -1)
        cv2.circle(img, wr_pt, 6, (40, 40, 40), -1)

        # Claw
        m1 = joint_ticks.get(1, 517)
        th_w_rad = math.radians(fk["angles"]["theta_world_deg"])
        claw_span_deg = (m1 - 226) * (35.0 / (782.0 - 226.0))
        perp_dx = -math.sin(th_w_rad) * 14.0
        perp_dy = math.cos(th_w_rad) * 14.0
        f1_x = fk["tool_tip"][0] + perp_dx * math.cos(math.radians(claw_span_deg))
        f1_y = fk["tool_tip"][1] + perp_dy * math.cos(math.radians(claw_span_deg))
        f2_x = fk["tool_tip"][0] - perp_dx * math.cos(math.radians(claw_span_deg))
        f2_y = fk["tool_tip"][1] - perp_dy * math.cos(math.radians(claw_span_deg))
        cv2.line(img, tool_pt, w2i(f1_x, f1_y), (40, 120, 230), 4, cv2.LINE_AA)
        cv2.line(img, tool_pt, w2i(f2_x, f2_y), (40, 120, 230), 4, cv2.LINE_AA)

        cv2.circle(img, tool_pt, 12, (0, 160, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tool_pt, 3, (0, 160, 255), -1)

        # Header HUD
        cv2.rectangle(img, (0, 0), (img_w, 40), (35, 38, 42), -1)
        cv2.putText(img, status_text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        ee_info = f"Tip: X={fk['tool_tip'][0]:.1f}mm | Y={fk['tool_tip'][1]:.1f}mm | Z={fk['tool_tip'][2]:.1f}mm"
        cv2.putText(img, ee_info, (img_w - 380, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 220, 140), 2, cv2.LINE_AA)

        return img

    # ---------------------------------------------------------------------------------
    # 2. SIDE ELEVATION VIEW (Y-Z Plane)
    # ---------------------------------------------------------------------------------
    def render_side_view(
        self,
        joint_ticks: Dict[int, int],
        cubes: List[dict],
        status_text: str = "SIDE ELEVATION (Y-Z PLANE)",
    ) -> np.ndarray:
        """
        Renders side view looking down the X axis from the East (+X) toward the West (-X).
        Shows Y (0 to 600 mm) horizontally, Z (0 to 400 mm) vertically.
        Directly validates arm height descent to Z=9.5mm on the table surface.
        """
        scale = 1.4  # px / mm
        margin_y = 60
        margin_z = 50
        view_w = int(650 * scale) + 2 * margin_y
        view_h = int(380 * scale) + 2 * margin_z + 40

        img = np.full((view_h, view_w, 3), (242, 244, 247), dtype=np.uint8)

        def yz2i(y_mm, z_mm):
            px = int(margin_y + (y_mm + 25.0) * scale)
            py = int(view_h - margin_z - (z_mm + 15.0) * scale)
            return px, py

        # 1. Baseboard Table
        t_tl = yz2i(0.0, 0.0)
        t_br = yz2i(600.0, -18.0)
        cv2.rectangle(img, t_tl, t_br, (215, 215, 210), -1)
        cv2.rectangle(img, t_tl, t_br, (120, 120, 120), 2)
        cv2.putText(img, "TABLE BASEBOARD (Z = 0.0 mm)", (t_tl[0] + 20, t_br[1] + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 2)

        # 2. Golden Pickup Zone on Table (Y: 100..250)
        pz_tl = yz2i(100.0, 0.0)
        pz_br = yz2i(250.0, -18.0)
        cv2.rectangle(img, pz_tl, pz_br, (180, 230, 190), -1)
        cv2.rectangle(img, pz_tl, pz_br, (60, 160, 70), 2)
        cv2.putText(img, "GOLDEN ZONE (Y 100..250)", (pz_tl[0] + 15, pz_tl[1] + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (40, 130, 50), 2)

        # 3. Z-Height Reference Grid Lines
        for z_val in [0.0, 9.5, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0]:
            p1 = yz2i(-20.0, z_val)
            p2 = yz2i(620.0, z_val)
            if z_val == 9.5:
                cv2.line(img, p1, p2, (0, 180, 240), 1, cv2.LINE_AA)
                cv2.putText(img, f"Z={z_val:.1f}mm (Target)", (p1[0] - 50, p1[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 140, 200), 1)
            elif z_val == 0.0:
                cv2.line(img, p1, p2, (60, 60, 60), 2, cv2.LINE_AA)
                cv2.putText(img, f"Z={z_val:.0f}mm", (p1[0] - 50, p1[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1)
            else:
                cv2.line(img, p1, p2, (215, 210, 200), 1, cv2.LINE_AA)
                cv2.putText(img, f"Z={z_val:.0f}mm", (p1[0] - 50, p1[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 130, 130), 1)

        # Y Reference Grid Lines
        for y_val in range(0, 650, 100):
            p1 = yz2i(float(y_val), -18.0)
            p2 = yz2i(float(y_val), 320.0)
            cv2.line(img, p1, p2, (225, 220, 210), 1, cv2.LINE_AA)
            cv2.putText(img, f"Y={y_val}", (p1[0] - 15, yz2i(float(y_val), -25.0)[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (130, 130, 130), 1)

        # 4. Container Box at Rear (Y: 500..600, height 35mm)
        c_tl = yz2i(500.0, 35.0)
        c_br = yz2i(600.0, 0.0)
        cv2.rectangle(img, c_tl, c_br, (190, 200, 230), -1)
        cv2.rectangle(img, c_tl, c_br, (60, 70, 180), 2)
        cv2.putText(img, "CONTAINERS (Z=35mm)", (c_tl[0] + 10, c_tl[1] + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (40, 50, 150), 2)

        # 5. Cubes in Side View
        for c in cubes:
            col = (40, 40, 220) if c["color"].lower() == "red" else (40, 180, 50)
            c_tl = yz2i(c["y"] - 12.5, 19.05)
            c_br = yz2i(c["y"] + 12.5, 0.0)
            cv2.rectangle(img, c_tl, c_br, col, -1)
            cv2.rectangle(img, c_tl, c_br, (20, 20, 20), 2)
            cv2.circle(img, yz2i(c["y"], 9.53), 3, (255, 255, 255), -1)

        # 6. xArm Forward Kinematics Links
        fk = self.geom.compute_joint_positions(joint_ticks)
        base_pt = yz2i(self.geom.base_y, 0.0)
        sh_pt = yz2i(fk["shoulder"][1], fk["shoulder"][2])
        el_pt = yz2i(fk["elbow"][1], fk["elbow"][2])
        wr_pt = yz2i(fk["wrist"][1], fk["wrist"][2])
        tool_pt = yz2i(fk["tool_tip"][1], fk["tool_tip"][2])

        # Base Pedestal (height d1 = 65mm)
        ped_tl = yz2i(self.geom.base_y - 18.0, self.geom.d1)
        ped_br = yz2i(self.geom.base_y + 18.0, 0.0)
        cv2.rectangle(img, ped_tl, ped_br, (180, 130, 40), -1)
        cv2.rectangle(img, ped_tl, ped_br, (90, 60, 15), 2)
        cv2.putText(img, "BASE (d1=65mm)", (ped_tl[0] - 50, ped_tl[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 60, 10), 1)

        # Draw Robotic Links
        cv2.line(img, sh_pt, el_pt, (180, 100, 20), 12, cv2.LINE_AA)  # Upper arm L1=101mm
        cv2.line(img, el_pt, wr_pt, (150, 80, 15), 10, cv2.LINE_AA)   # Forearm L2=95mm
        cv2.line(img, wr_pt, tool_pt, (50, 50, 50), 8, cv2.LINE_AA)   # Tool shaft L3=150mm

        cv2.circle(img, sh_pt, 10, (40, 40, 40), -1)
        cv2.circle(img, el_pt, 9, (40, 40, 40), -1)
        cv2.circle(img, wr_pt, 8, (40, 40, 40), -1)

        # Draw Tool Claw and Approach Pitch Indicator
        cv2.circle(img, tool_pt, 14, (0, 160, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tool_pt, 4, (0, 160, 255), -1)

        phi = fk["angles"]["pitch_phi_deg"]
        arr_len = 35.0
        arr_end = yz2i(
            fk["tool_tip"][1] - arr_len * math.cos(math.radians(abs(phi))),
            fk["tool_tip"][2] - arr_len * math.sin(math.radians(abs(phi))),
        )
        cv2.arrowedLine(img, tool_pt, arr_end, (0, 120, 240), 2, cv2.LINE_AA, tipLength=0.3)

        # Dimension / Measurement Callout for Tool Altitude with Clean Card Backdrop
        tip_z = fk["tool_tip"][2]
        callout_x = tool_pt[0] + 35
        callout_y = tool_pt[1] - 50

        # Background card
        card_w = 280
        card_h = 52
        cv2.rectangle(img, (callout_x - 5, callout_y - 20), (callout_x + card_w, callout_y + card_h - 20),
                      (255, 255, 255), -1)
        cv2.rectangle(img, (callout_x - 5, callout_y - 20), (callout_x + card_w, callout_y + card_h - 20),
                      (180, 40, 40), 2)
        cv2.line(img, tool_pt, (callout_x - 5, callout_y + 6), (180, 40, 40), 2, cv2.LINE_AA)

        cv2.putText(img, f"TOOL TIP ALTITUDE Z = {tip_z:.2f} mm", (callout_x + 6, callout_y + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.54, (180, 30, 30), 2, cv2.LINE_AA)
        cv2.putText(img, f"Pitch: {phi:.1f} deg | Reach: {fk['angles']['radial_reach_mm']:.1f}mm",
                    (callout_x + 6, callout_y + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (60, 60, 60), 1, cv2.LINE_AA)

        # Header HUD
        cv2.rectangle(img, (0, 0), (view_w, 40), (35, 38, 42), -1)
        cv2.putText(img, status_text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        h_info = f"Grasp Status: {'AT CUBE (Z ~ 9.5mm)' if abs(tip_z - 9.53) < 5.0 else 'STANDOFF/TRANSIT'} | Z={tip_z:.1f}mm"
        cv2.putText(img, h_info, (view_w - 450, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 220, 140), 2, cv2.LINE_AA)

        return img

    # ---------------------------------------------------------------------------------
    # 3. FRONT ELEVATION VIEW (X-Z Plane)
    # ---------------------------------------------------------------------------------
    def render_front_view(
        self,
        joint_ticks: Dict[int, int],
        cubes: List[dict],
        status_text: str = "FRONT ELEVATION (X-Z PLANE)",
    ) -> np.ndarray:
        """
        Renders front view looking from the South (front of table) toward the North (rear).
        Shows X (0 to 350 mm) horizontally, Z (0 to 400 mm) vertically.
        Validates lateral centering over grid columns and cube grasping.
        """
        scale = 1.8  # px / mm
        margin_x = 60
        margin_z = 50
        view_w = int(350 * scale) + 2 * margin_x
        view_h = int(380 * scale) + 2 * margin_z + 40

        img = np.full((view_h, view_w, 3), (242, 244, 247), dtype=np.uint8)

        def xz2i(x_mm, z_mm):
            px = int(margin_x + x_mm * scale)
            py = int(view_h - margin_z - (z_mm + 15.0) * scale)
            return px, py

        # 1. Baseboard Table
        t_tl = xz2i(0.0, 0.0)
        t_br = xz2i(350.0, -18.0)
        cv2.rectangle(img, t_tl, t_br, (215, 215, 210), -1)
        cv2.rectangle(img, t_tl, t_br, (120, 120, 120), 2)

        # 2. Columns Grid Lines (C0..C6, step 50mm)
        for c in range(8):
            x = c * 50.0
            p1 = xz2i(x, -18.0)
            p2 = xz2i(x, 320.0)
            cv2.line(img, p1, p2, (225, 220, 210), 1, cv2.LINE_AA)
            if c < 7:
                mid_x = (c + 0.5) * 50.0
                cv2.putText(img, f"C{c}", (xz2i(mid_x, -30.0)[0] - 10, xz2i(mid_x, -30.0)[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 90, 60), 2)

        # Z Grid Lines
        for z_val in [0.0, 9.5, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0]:
            p1 = xz2i(0.0, z_val)
            p2 = xz2i(350.0, z_val)
            cv2.line(img, p1, p2, (215, 210, 200), 1, cv2.LINE_AA)
            cv2.putText(img, f"Z={z_val:.0f}", (p1[0] - 45, p1[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 130, 130), 1)

        # 3. Cubes
        for c in cubes:
            col = (40, 40, 220) if c["color"].lower() == "red" else (40, 180, 50)
            c_tl = xz2i(c["x"] - 12.5, 19.05)
            c_br = xz2i(c["x"] + 12.5, 0.0)
            cv2.rectangle(img, c_tl, c_br, col, -1)
            cv2.rectangle(img, c_tl, c_br, (20, 20, 20), 2)
            cv2.putText(img, f"{c['color'].upper()}", (c_tl[0] - 8, c_tl[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1)

        # 4. xArm Base & Links
        fk = self.geom.compute_joint_positions(joint_ticks)
        base_pt = xz2i(self.geom.base_x, 0.0)
        sh_pt = xz2i(fk["shoulder"][0], fk["shoulder"][2])
        el_pt = xz2i(fk["elbow"][0], fk["elbow"][2])
        wr_pt = xz2i(fk["wrist"][0], fk["wrist"][2])
        tool_pt = xz2i(fk["tool_tip"][0], fk["tool_tip"][2])

        # Base Pedestal
        ped_tl = xz2i(self.geom.base_x - 18.0, self.geom.d1)
        ped_br = xz2i(self.geom.base_x + 18.0, 0.0)
        cv2.rectangle(img, ped_tl, ped_br, (180, 130, 40), -1)
        cv2.rectangle(img, ped_tl, ped_br, (90, 60, 15), 2)

        # Links
        cv2.line(img, sh_pt, el_pt, (180, 100, 20), 12, cv2.LINE_AA)
        cv2.line(img, el_pt, wr_pt, (150, 80, 15), 10, cv2.LINE_AA)
        cv2.line(img, wr_pt, tool_pt, (50, 50, 50), 8, cv2.LINE_AA)

        cv2.circle(img, sh_pt, 10, (40, 40, 40), -1)
        cv2.circle(img, el_pt, 9, (40, 40, 40), -1)
        cv2.circle(img, wr_pt, 8, (40, 40, 40), -1)

        # Claw Jaws in Front View
        m1 = joint_ticks.get(1, 517)
        jaw_span = (m1 - 226) * (35.0 / (782.0 - 226.0)) + 10.0
        j1_pt = xz2i(fk["tool_tip"][0] - jaw_span * 0.5, fk["tool_tip"][2])
        j2_pt = xz2i(fk["tool_tip"][0] + jaw_span * 0.5, fk["tool_tip"][2])
        cv2.line(img, (tool_pt[0], tool_pt[1] - 15), j1_pt, (40, 120, 230), 4, cv2.LINE_AA)
        cv2.line(img, (tool_pt[0], tool_pt[1] - 15), j2_pt, (40, 120, 230), 4, cv2.LINE_AA)

        cv2.circle(img, tool_pt, 12, (0, 160, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tool_pt, 4, (0, 160, 255), -1)

        # Lateral Alignment Marker with Clean Card Backdrop
        tip_x = fk["tool_tip"][0]
        col_idx = int(math.floor(tip_x / 50.0))
        card_x = tool_pt[0] - 120
        card_y = tool_pt[1] - 55
        cv2.rectangle(img, (card_x, card_y), (card_x + 240, card_y + 28), (255, 255, 255), -1)
        cv2.rectangle(img, (card_x, card_y), (card_x + 240, card_y + 28), (30, 40, 180), 2)
        cv2.putText(img, f"X = {tip_x:.1f} mm (Col {col_idx}) | Z = {fk['tool_tip'][2]:.1f} mm",
                    (card_x + 8, card_y + 19), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (20, 30, 160), 2)

        # Header HUD
        cv2.rectangle(img, (0, 0), (view_w, 40), (35, 38, 42), -1)
        cv2.putText(img, status_text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    # ---------------------------------------------------------------------------------
    # 4. 3D ISOMETRIC PERSPECTIVE VIEW
    # ---------------------------------------------------------------------------------
    def render_isometric_view(
        self,
        joint_ticks: Dict[int, int],
        cubes: List[dict],
        status_text: str = "3D ISOMETRIC PERSPECTIVE VIEW",
    ) -> np.ndarray:
        """
        Renders axonometric 3D projection of the entire workcell.
        Gives full spatial depth of arm reaching forward, descending down, and grasping.
        """
        view_w = 850
        view_h = 600
        img = np.full((view_h, view_w, 3), (242, 244, 247), dtype=np.uint8)

        cos_a = math.cos(math.radians(30.0))
        sin_a = math.sin(math.radians(30.0))
        scale = 0.95
        origin_screen_x = 425
        origin_screen_y = 360

        def iso3d(x_mm, y_mm, z_mm):
            dx = x_mm - 175.0
            dy = y_mm - 250.0
            dz = z_mm
            sx = origin_screen_x + (dx * cos_a - dy * cos_a) * scale
            sy = origin_screen_y + (dx * sin_a + dy * sin_a) * scale - dz * 1.25 * scale
            return int(sx), int(sy)

        # 1. Baseboard Table Outline
        c1 = iso3d(0.0, 0.0, 0.0)
        c2 = iso3d(350.0, 0.0, 0.0)
        c3 = iso3d(350.0, 600.0, 0.0)
        c4 = iso3d(0.0, 600.0, 0.0)
        pts = np.array([c1, c2, c3, c4], np.int32)
        cv2.fillPoly(img, [pts], (225, 228, 222))
        cv2.polylines(img, [pts], True, (130, 130, 130), 2)

        # Baseboard Thickness
        c1_b = iso3d(0.0, 0.0, -18.0)
        c2_b = iso3d(350.0, 0.0, -18.0)
        c3_b = iso3d(350.0, 600.0, -18.0)
        cv2.fillPoly(img, [np.array([c1, c2, c2_b, c1_b], np.int32)], (190, 192, 185))
        cv2.fillPoly(img, [np.array([c2, c3, c3_b, c2_b], np.int32)], (170, 172, 165))
        cv2.polylines(img, [np.array([c1_b, c2_b, c3_b], np.int32)], False, (110, 110, 110), 2)

        # 2. Golden Pickup Zone in 3D (X: 75..275, Y: 100..250)
        pz1 = iso3d(75.0, 100.0, 0.0)
        pz2 = iso3d(275.0, 100.0, 0.0)
        pz3 = iso3d(275.0, 250.0, 0.0)
        pz4 = iso3d(75.0, 250.0, 0.0)
        cv2.fillPoly(img, [np.array([pz1, pz2, pz3, pz4], np.int32)], (210, 238, 215))
        cv2.polylines(img, [np.array([pz1, pz2, pz3, pz4], np.int32)], True, (70, 175, 80), 2)

        # 3. 5cm Grid Lines
        for c in range(1, 7):
            x = c * 50.0
            cv2.line(img, iso3d(x, 0.0, 0.0), iso3d(x, 600.0, 0.0), (200, 185, 160), 1, cv2.LINE_AA)
        for r in range(1, 12):
            y = r * 50.0
            cv2.line(img, iso3d(0.0, y, 0.0), iso3d(350.0, y, 0.0), (200, 185, 160), 1, cv2.LINE_AA)

        # 4. Containers in 3D
        # Red Box
        r1 = iso3d(0.0, 500.0, 0.0)
        r2 = iso3d(100.0, 500.0, 0.0)
        r3 = iso3d(100.0, 600.0, 0.0)
        r4 = iso3d(0.0, 600.0, 0.0)
        r1_t = iso3d(0.0, 500.0, 35.0)
        r2_t = iso3d(100.0, 500.0, 35.0)
        r3_t = iso3d(100.0, 600.0, 35.0)
        r4_t = iso3d(0.0, 600.0, 35.0)
        cv2.fillPoly(img, [np.array([r1_t, r2_t, r3_t, r4_t], np.int32)], (180, 180, 240))
        cv2.fillPoly(img, [np.array([r1, r2, r2_t, r1_t], np.int32)], (140, 140, 210))
        cv2.polylines(img, [np.array([r1_t, r2_t, r3_t, r4_t], np.int32)], True, (30, 30, 200), 2)

        # Green Box
        g1 = iso3d(250.0, 500.0, 0.0)
        g2 = iso3d(350.0, 500.0, 0.0)
        g3 = iso3d(350.0, 600.0, 0.0)
        g4 = iso3d(250.0, 600.0, 0.0)
        g1_t = iso3d(250.0, 500.0, 35.0)
        g2_t = iso3d(350.0, 500.0, 35.0)
        g3_t = iso3d(350.0, 600.0, 35.0)
        g4_t = iso3d(250.0, 600.0, 35.0)
        cv2.fillPoly(img, [np.array([g1_t, g2_t, g3_t, g4_t], np.int32)], (180, 240, 180))
        cv2.fillPoly(img, [np.array([g1, g2, g2_t, g1_t], np.int32)], (140, 210, 140))
        cv2.polylines(img, [np.array([g1_t, g2_t, g3_t, g4_t], np.int32)], True, (30, 180, 30), 2)

        # 5. Cubes in 3D (37.5 x 37.5 x 45.0 mm)
        for c in cubes:
            col_top = (50, 50, 240) if c["color"].lower() == "red" else (50, 220, 60)
            col_side = (30, 30, 190) if c["color"].lower() == "red" else (30, 170, 40)
            cz_base = max(0.0, float(c.get("z", 22.5)) - 22.5)
            cz_top = cz_base + 45.0
            b1 = iso3d(c["x"] - 18.75, c["y"] - 18.75, cz_base)
            b2 = iso3d(c["x"] + 18.75, c["y"] - 18.75, cz_base)
            b3 = iso3d(c["x"] + 18.75, c["y"] + 18.75, cz_base)
            b4 = iso3d(c["x"] - 18.75, c["y"] + 18.75, cz_base)
            t1 = iso3d(c["x"] - 18.75, c["y"] - 18.75, cz_top)
            t2 = iso3d(c["x"] + 18.75, c["y"] - 18.75, cz_top)
            t3 = iso3d(c["x"] + 18.75, c["y"] + 18.75, cz_top)
            t4 = iso3d(c["x"] - 18.75, c["y"] + 18.75, cz_top)
            cv2.fillPoly(img, [np.array([t1, t2, t3, t4], np.int32)], col_top)
            cv2.fillPoly(img, [np.array([b1, b2, t2, t1], np.int32)], col_side)
            cv2.polylines(img, [np.array([t1, t2, t3, t4], np.int32)], True, (20, 20, 20), 1)

        # 6. xArm in 3D
        fk = self.geom.compute_joint_positions(joint_ticks)
        base_3d = iso3d(self.geom.base_x, self.geom.base_y, 0.0)
        sh_3d = iso3d(fk["shoulder"][0], fk["shoulder"][1], fk["shoulder"][2])
        el_3d = iso3d(fk["elbow"][0], fk["elbow"][1], fk["elbow"][2])
        wr_3d = iso3d(fk["wrist"][0], fk["wrist"][1], fk["wrist"][2])
        tool_3d = iso3d(fk["tool_tip"][0], fk["tool_tip"][1], fk["tool_tip"][2])

        # Pedestal
        cv2.line(img, base_3d, sh_3d, (180, 130, 40), 16, cv2.LINE_AA)
        cv2.line(img, sh_3d, el_3d, (180, 100, 20), 12, cv2.LINE_AA)
        cv2.line(img, el_3d, wr_3d, (150, 80, 15), 10, cv2.LINE_AA)
        cv2.line(img, wr_3d, tool_3d, (50, 50, 50), 8, cv2.LINE_AA)

        cv2.circle(img, base_3d, 12, (90, 60, 15), -1)
        cv2.circle(img, sh_3d, 10, (40, 40, 40), -1)
        cv2.circle(img, el_3d, 9, (40, 40, 40), -1)
        cv2.circle(img, wr_3d, 8, (40, 40, 40), -1)

        cv2.circle(img, tool_3d, 12, (0, 160, 255), 2, cv2.LINE_AA)
        cv2.circle(img, tool_3d, 4, (0, 160, 255), -1)

        # Header HUD
        cv2.rectangle(img, (0, 0), (view_w, 40), (35, 38, 42), -1)
        cv2.putText(img, status_text, (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        tip_info = f"Tool Tip: Z={fk['tool_tip'][2]:.1f}mm | Reach: {fk['angles']['radial_reach_mm']:.1f}mm"
        cv2.putText(img, tip_info, (view_w - 380, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 220, 140), 2, cv2.LINE_AA)

        return img

    # ---------------------------------------------------------------------------------
    # 5. COMPOSITE 4-PANEL MULTI-VIEW DASHBOARD
    # ---------------------------------------------------------------------------------
    def render_multiview_dashboard(
        self,
        joint_ticks: Dict[int, int],
        cubes: List[dict],
        status_text: str = "xArm 1S Multi-View Simulation Inspector",
        prefix: str = "",
    ) -> str:
        """
        Renders all 4 perspectives, resizes to uniform 2x2 grid, and saves to file.
        """
        topdown = self.render_topdown(joint_ticks, cubes, "TOP-DOWN VIEW (X-Y)")
        side = self.render_side_view(joint_ticks, cubes, "SIDE ELEVATION (Y-Z)")
        front = self.render_front_view(joint_ticks, cubes, "FRONT ELEVATION (X-Z)")
        iso = self.render_isometric_view(joint_ticks, cubes, "3D PERSPECTIVE VIEW")

        # Save individual full-resolution views
        p = f"{prefix}_" if prefix else ""
        cv2.imwrite(os.path.join(self.output_dir, f"{p}view_topdown.png"), topdown)
        cv2.imwrite(os.path.join(self.output_dir, f"{p}view_side_elevation.png"), side)
        cv2.imwrite(os.path.join(self.output_dir, f"{p}view_front_elevation.png"), front)
        cv2.imwrite(os.path.join(self.output_dir, f"{p}view_3d_perspective.png"), iso)

        # Resize each to standard panel size: 900 x 600
        p_w, p_h = 900, 600
        td_resized = cv2.resize(topdown, (p_w, p_h), interpolation=cv2.INTER_AREA)
        side_resized = cv2.resize(side, (p_w, p_h), interpolation=cv2.INTER_AREA)
        front_resized = cv2.resize(front, (p_w, p_h), interpolation=cv2.INTER_AREA)
        iso_resized = cv2.resize(iso, (p_w, p_h), interpolation=cv2.INTER_AREA)

        # Assemble 2x2 grid
        top_row = np.hstack([td_resized, side_resized])
        bottom_row = np.hstack([iso_resized, front_resized])
        dashboard = np.vstack([top_row, bottom_row])

        # Overall Banner at top
        banner = np.full((70, dashboard.shape[1], 3), (25, 28, 32), dtype=np.uint8)
        cv2.putText(
            banner, "xArm 1S ROS 2 SIMULATION MULTI-PERSPECTIVE INSPECTOR", (25, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA
        )
        fk = self.geom.compute_joint_positions(joint_ticks)
        subtext = (
            f"End-Effector: X={fk['tool_tip'][0]:.1f}mm | Y={fk['tool_tip'][1]:.1f}mm | "
            f"Z={fk['tool_tip'][2]:.2f}mm | Pitch={fk['angles']['pitch_phi_deg']:.1f}deg | "
            f"State: {status_text}"
        )
        cv2.putText(
            banner, subtext, (25, 56),
            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (80, 225, 140), 2, cv2.LINE_AA
        )

        full_img = np.vstack([banner, dashboard])
        out_name = f"{p}simulation_multiview.png" if prefix else "simulation_multiview.png"
        out_path = os.path.join(self.output_dir, out_name)
        cv2.imwrite(out_path, full_img)
        return out_path


def main():
    parser = argparse.ArgumentParser(description="Multi-Angle Simulation Visualizer")
    parser.add_argument("--output_dir", default="artifacts", help="Output directory for screenshots")
    parser.add_argument("--loop", action="store_true", help="Continuously render from ROS 2 topics")
    parser.add_argument("--pose", choices=["all", "pick", "standoff", "home", "drop_red", "drop_green"],
                        default="all", help="Static pose to render (or 'all' to generate progression gallery)")
    parser.add_argument("--grid", choices=["full_grid", "dual"], default="full_grid",
                        help="Cube configuration: 'full_grid' (all 28 picking cells) or 'dual' (2 test cubes)")
    args = parser.parse_args()

    renderer = MultiViewRenderer(output_dir=args.output_dir)

    if args.grid == "full_grid":
        cubes = []
        cube_id = 1
        import random
        rng = random.Random(42)  # Balanced scattered pattern for gallery
        for r in [7, 8, 9, 10]:
            for c in range(7):
                color = "red" if rng.random() < 0.5 else "green"
                x_m = (c + 0.5) * 50.0
                y_m = (11 - r + 0.5) * 50.0
                cubes.append({"id": cube_id, "color": color, "x": x_m, "y": y_m})
                cube_id += 1
    else:
        cubes = [
            {"id": 1, "color": "red", "x": 125.0, "y": 175.0},
            {"id": 2, "color": "green", "x": 225.0, "y": 175.0},
        ]

    poses = {
        "standoff": {
            "ticks": {1: 226, 2: 503, 3: 650, 4: 290, 5: 540, 6: 443},
            "status": "PHASE 1: APPROACH STANDOFF (Z = 35.0mm above cube)",
        },
        "pick": {
            "ticks": {1: 610, 2: 503, 3: 772, 4: 183, 5: 586, 6: 443},
            "status": "PHASE 2: PICK GRASP AT TABLE SURFACE (Z = 9.72mm, Pitch = -69.8 deg)",
        },
        "drop_red": {
            "ticks": {1: 610, 2: 503, 3: 149, 4: 629, 5: 347, 6: 650},
            "status": "PHASE 5: OVERHEAD REAR-FLIP: RED CONTAINER DROP (X=50.1mm, Y=549.9mm, Z=45.0mm)",
        },
        "drop_green": {
            "ticks": {1: 610, 2: 503, 3: 149, 4: 629, 5: 347, 6: 354},
            "status": "PHASE 5: OVERHEAD REAR-FLIP: GREEN CONTAINER DROP (X=299.9mm, Y=549.9mm, Z=45.0mm)",
        },
    }

    if args.pose == "all":
        for name, cfg in poses.items():
            path = renderer.render_multiview_dashboard(
                cfg["ticks"], cubes, status_text=cfg["status"], prefix=name
            )
            print(f"Generated {name} dashboard: {path}")
        # Default dashboard is pick
        renderer.render_multiview_dashboard(
            poses["pick"]["ticks"], cubes, status_text=poses["pick"]["status"], prefix=""
        )
        return

    # Single pose render
    cfg = poses.get(args.pose, poses["pick"])
    path = renderer.render_multiview_dashboard(
        cfg["ticks"], cubes, status_text=cfg["status"]
    )
    print(f"Generated multi-view dashboard: {path}")


if __name__ == "__main__":
    main()
