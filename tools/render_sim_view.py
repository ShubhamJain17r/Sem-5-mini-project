#!/usr/bin/env python3
"""
Simulated Top-Down Overhead Camera & Visualizer Renderer for xArm 1S.

Renders an overhead top-down visual view of the 7x12 arena, 5cm grid,
xArm links, claw, and red/green cubes using OpenCV, and saves it to an image file.

This allows the AI agent to visually inspect the simulation state at any moment.

Usage:
  ./docker/run.sh python3 tools/render_sim_view.py
  (or with --loop to continuously update artifacts/simulation_topdown.png at 2 Hz)
"""

import argparse
import math
import os
import sys
import time

try:
    import cv2
    import numpy as np
except ImportError:
    print("OpenCV or NumPy not installed. Run inside Docker container.")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from sorting_arm.kinematics import (
        DEG_PER_TICK,
        load_kinematic_parameters,
    )
    from visualization_msgs.msg import MarkerArray
except ImportError:
    rclpy = None


class SimViewRenderer:
    """Renders top-down 2D projection of xArm workcell to an OpenCV BGR image."""

    def __init__(self, output_path: str = "artifacts/simulation_topdown.png"):
        self.output_path = output_path
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        # Scale: 2 pixels per mm
        # 350mm width -> 700px, 600mm height -> 1200px + margins
        self.scale = 2.0  # px / mm
        self.margin_x = 80
        self.margin_y = 80
        self.arena_w_px = int(350 * self.scale)
        self.arena_h_px = int(600 * self.scale)
        self.img_w = self.arena_w_px + 2 * self.margin_x
        self.img_h = self.arena_h_px + 2 * self.margin_y

        # Physical constants
        self.base_x = 175.0
        self.base_y = 375.0
        self.L1 = 101.0
        self.L2 = 95.0
        self.L3 = 150.0

    def world_to_img(self, x_mm: float, y_mm: float):
        """Converts world coordinates (mm) to image pixel coordinates."""
        # World: X in [0, 350] (left to right), Y in [0, 600] (front to rear)
        # Image: Y=0 (Front) is at bottom, Y=600 (Rear) is at top
        px = int(self.margin_x + x_mm * self.scale)
        py = int(self.margin_y + (600.0 - y_mm) * self.scale)
        return px, py

    def render(
        self,
        joint_ticks: dict,
        cubes: list,
        ee_pos: tuple = None,
        status_text: str = "SIMULATION ACTIVE",
    ) -> str:
        """Renders the top-down frame and writes to image file."""
        img = np.full((self.img_h, self.img_w, 3), (245, 245, 245), dtype=np.uint8)

        # 1. Baseboard Table
        tl_x, tl_y = self.world_to_img(0.0, 600.0)
        br_x, br_y = self.world_to_img(350.0, 0.0)
        cv2.rectangle(img, (tl_x, tl_y), (br_x, br_y), (225, 225, 220), -1)
        cv2.rectangle(img, (tl_x, tl_y), (br_x, br_y), (140, 140, 140), 2)

        # 2. Picking Zone (Rows 7..10: Y in [100, 300], X in [25, 325])
        pz_tl = self.world_to_img(25.0, 300.0)
        pz_br = self.world_to_img(325.0, 100.0)
        cv2.rectangle(img, pz_tl, pz_br, (215, 240, 215), -1)
        cv2.rectangle(img, pz_tl, pz_br, (60, 180, 75), 2)
        cv2.putText(
            img, "PICKING ZONE (Rows 7..10)", (pz_tl[0] + 120, pz_br[1] + 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (40, 140, 50), 2, cv2.LINE_AA
        )

        # 3. 5cm Grid Lines & Labels
        # Vertical lines (constant X, step 50mm)
        for c in range(8):
            x = c * 50.0
            p1 = self.world_to_img(x, 0.0)
            p2 = self.world_to_img(x, 600.0)
            cv2.line(img, p1, p2, (200, 180, 140), 1, cv2.LINE_AA)
            if c < 7:
                mid_x = (c + 0.5) * 50.0
                lbl_pt = self.world_to_img(mid_x, -12.0)
                cv2.putText(img, f"C{c}", (lbl_pt[0] - 12, lbl_pt[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 90, 60), 2, cv2.LINE_AA)

        # Horizontal lines (constant Y, step 50mm)
        for r_idx in range(13):
            y = r_idx * 50.0
            p1 = self.world_to_img(0.0, y)
            p2 = self.world_to_img(350.0, y)
            cv2.line(img, p1, p2, (200, 180, 140), 1, cv2.LINE_AA)
            if r_idx < 12:
                # Row 11 is front (y=0..50), Row 0 is rear (y=550..600)
                row_num = 11 - r_idx
                mid_y = (r_idx + 0.5) * 50.0
                lbl_pt = self.world_to_img(-15.0, mid_y)
                cv2.putText(img, f"R{row_num}", (lbl_pt[0] - 30, lbl_pt[1] + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 90, 60), 2, cv2.LINE_AA)

        # 4. Container Boxes
        # Red Box at (50, 550, size 100x100)
        r_tl = self.world_to_img(0.0, 600.0)
        r_br = self.world_to_img(100.0, 500.0)
        cv2.rectangle(img, r_tl, r_br, (180, 180, 240), -1)
        cv2.rectangle(img, r_tl, r_br, (40, 40, 220), 2)
        cv2.putText(img, "RED TRAY", (r_tl[0] + 15, r_tl[1] + 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 180), 2, cv2.LINE_AA)

        # Green Box at (300, 550, size 100x100)
        g_tl = self.world_to_img(250.0, 600.0)
        g_br = self.world_to_img(350.0, 500.0)
        cv2.rectangle(img, g_tl, g_br, (180, 240, 180), -1)
        cv2.rectangle(img, g_tl, g_br, (40, 180, 40), 2)
        cv2.putText(img, "GREEN TRAY", (g_tl[0] + 10, g_tl[1] + 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 140, 20), 2, cv2.LINE_AA)

        # 5. Visual Cubes
        for c in cubes:
            cx, cy = self.world_to_img(c["x"], c["y"])
            c_size = int(25.4 * self.scale)
            half = c_size // 2
            b_color = (30, 30, 220) if c["color"].lower() == "red" else (30, 180, 40)
            cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), b_color, -1)
            cv2.rectangle(img, (cx - half, cy - half), (cx + half, cy + half), (20, 20, 20), 2)
            cv2.putText(img, f"{c['color'].upper()} #{c['id']}", (cx - half - 10, cy - half - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 2, cv2.LINE_AA)

        # 6. xArm 1S Manipulator Top-Down Projection
        base_pt = self.world_to_img(self.base_x, self.base_y)
        cv2.circle(img, base_pt, 22, (200, 120, 30), -1)  # Blue chassis hub
        cv2.circle(img, base_pt, 22, (100, 60, 10), 2)
        cv2.circle(img, base_pt, 8, (40, 40, 40), -1)

        # Compute FK link projections using exact kinematics parameters
        p = load_kinematic_parameters(arm_id="xarm_1")
        dir_mult = p.direction_multipliers or {i: 1 for i in range(1, 7)}
        centers = p.nominal_center_ticks or {i: 500 for i in range(1, 7)}

        m6 = joint_ticks.get(6, centers.get(6, 502))
        m5 = joint_ticks.get(5, centers.get(5, 501))
        m4 = joint_ticks.get(4, centers.get(4, 496))
        m3 = joint_ticks.get(3, centers.get(3, 504))
        m1 = joint_ticks.get(1, centers.get(1, 226))

        theta_arm = p.base_zero_deg + dir_mult.get(6, 1) * (m6 - centers.get(6, 502)) * DEG_PER_TICK
        theta_world_deg = theta_arm + p.base_heading_deg  # -90° (South)
        theta_world_rad = math.radians(theta_world_deg)

        th_sh_deg = p.shoulder_zero_deg - dir_mult.get(5, 1) * (m5 - centers.get(5, 501)) * DEG_PER_TICK
        th_sh_rad = math.radians(th_sh_deg)

        alpha3_deg = p.elbow_zero_deg + dir_mult.get(4, 1) * (m4 - centers.get(4, 496)) * DEG_PER_TICK
        th_el_deg = th_sh_deg - (180.0 - alpha3_deg)
        th_el_rad = math.radians(th_el_deg)

        th_wr_rel = p.wrist_zero_deg - dir_mult.get(3, 1) * (m3 - centers.get(3, 504)) * DEG_PER_TICK
        phi_deg = th_el_deg + th_wr_rel
        phi_rad = math.radians(phi_deg)

        # 2D Projected radial distances along table
        r1 = self.L1 * math.cos(th_sh_rad)
        r2 = self.L2 * math.cos(th_el_rad)
        r3 = self.L3 * math.cos(phi_rad)

        cos_w = math.cos(theta_world_rad)
        sin_w = math.sin(theta_world_rad)

        x_elbow = self.base_x + r1 * cos_w
        y_elbow = self.base_y + r1 * sin_w
        x_wrist = x_elbow + r2 * cos_w
        y_wrist = y_elbow + r2 * sin_w
        x_ee = x_wrist + r3 * cos_w
        y_ee = y_wrist + r3 * sin_w

        elbow_pt = self.world_to_img(x_elbow, y_elbow)
        wrist_pt = self.world_to_img(x_wrist, y_wrist)
        ee_pt = self.world_to_img(x_ee, y_ee)

        # Draw links
        cv2.line(img, base_pt, elbow_pt, (180, 100, 20), 10, cv2.LINE_AA)   # Upper arm
        cv2.line(img, elbow_pt, wrist_pt, (150, 80, 15), 8, cv2.LINE_AA)   # Forearm
        cv2.line(img, wrist_pt, ee_pt, (50, 50, 50), 6, cv2.LINE_AA)       # Tool shaft

        cv2.circle(img, elbow_pt, 9, (40, 40, 40), -1)
        cv2.circle(img, wrist_pt, 7, (40, 40, 40), -1)

        # Draw claw fingers
        claw_span_deg = (m1 - 226) * (40.0 / (782.0 - 226.0))  # 0 to 40 deg
        claw_rad = math.radians(claw_span_deg)
        perp_dx = -sin_w * 16.0
        perp_dy = cos_w * 16.0

        f1_x = x_ee + perp_dx * math.cos(claw_rad)
        f1_y = y_ee + perp_dy * math.cos(claw_rad)
        f2_x = x_ee - perp_dx * math.cos(claw_rad)
        f2_y = y_ee - perp_dy * math.cos(claw_rad)

        f1_pt = self.world_to_img(f1_x, f1_y)
        f2_pt = self.world_to_img(f2_x, f2_y)
        cv2.line(img, ee_pt, f1_pt, (40, 120, 230), 4, cv2.LINE_AA)  # Left finger (orange pad)
        cv2.line(img, ee_pt, f2_pt, (40, 120, 230), 4, cv2.LINE_AA)  # Right finger (orange pad)

        # End effector crosshair & hover shadow
        cv2.circle(img, ee_pt, 14, (0, 160, 255), 2, cv2.LINE_AA)
        cv2.circle(img, ee_pt, 3, (0, 160, 255), -1)

        # 7. Real-Time HUD Overlay
        cv2.rectangle(img, (0, 0), (self.img_w, 65), (35, 38, 42), -1)
        col_idx = int(math.floor(x_ee / 50.0))
        row_idx = 11 - int(math.floor(y_ee / 50.0))
        col_idx = max(0, min(6, col_idx))
        row_idx = max(0, min(11, row_idx))

        claw_lbl = "CLOSED" if m1 >= 570 else "OPEN"
        hud_line1 = f"xArm Tool Tip: X={x_ee:.1f}mm | Y={y_ee:.1f}mm | Cell: [Col {col_idx}, Row {row_idx}]"
        hud_line2 = f"Claw: {claw_lbl} ({m1} ticks)  |  Status: {status_text}"

        cv2.putText(img, hud_line1, (20, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, hud_line2, (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 220, 120), 2, cv2.LINE_AA)

        cv2.imwrite(self.output_path, img)
        return self.output_path


def main():
    parser = argparse.ArgumentParser(description="Render top-down visual view of xArm simulation")
    parser.add_argument("--output", default="artifacts/simulation_topdown.png", help="Output PNG path")
    parser.add_argument("--loop", action="store_true", help="Continuously render from ROS 2 topics")
    args = parser.parse_args()

    renderer = SimViewRenderer(output_path=args.output)

    if not args.loop or rclpy is None:
        # Default snapshot render
        sample_cubes = [
            {"id": 1, "color": "red", "x": 130.0, "y": 200.0},
            {"id": 2, "color": "green", "x": 220.0, "y": 200.0},
        ]
        sample_ticks = {1: 226, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
        out = renderer.render(sample_ticks, sample_cubes, status_text="STANDALONE TOP-DOWN SNAPSHOT")
        print(f"Rendered simulation view to {out}")
        return

    # ROS 2 Live Loop
    rclpy.init()
    node = Node("sim_view_renderer")
    current_ticks = {1: 226, 2: 503, 3: 504, 4: 496, 5: 501, 6: 502}
    cubes = [
        {"id": 1, "color": "red", "x": 130.0, "y": 200.0},
        {"id": 2, "color": "green", "x": 220.0, "y": 200.0},
    ]

    def joint_cb(msg: JointState):
        for name, rad in zip(msg.name, msg.position):
            if "base" in name:
                current_ticks[6] = int(round(502 + math.degrees(rad) * (1000.0 / 240.0)))
            elif "shoulder" in name:
                current_ticks[5] = int(round(501 + math.degrees(rad) * (1000.0 / 240.0)))
            elif "elbow" in name:
                current_ticks[4] = int(round(496 + math.degrees(rad) * (1000.0 / 240.0)))
            elif "wrist_pitch" in name:
                current_ticks[3] = int(round(504 + math.degrees(rad) * (1000.0 / 240.0)))
            elif "claw" in name:
                current_ticks[1] = int(round(517 + math.degrees(rad) * (1000.0 / 240.0)))

    node.create_subscription(JointState, "/xarm_1/joint_states", joint_cb, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            renderer.render(current_ticks, cubes)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
