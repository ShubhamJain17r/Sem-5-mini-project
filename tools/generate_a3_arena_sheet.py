#!/usr/bin/env python3
"""
Generate a 1:1 True-Scale A3 Master Template Sheet for Single xArm 1S Workcell.

Dimensions & Scale:
  - Standard A3 Portrait: 297.0 mm (Width) x 420.0 mm (Height)
  - Print Resolution: 300 DPI (11.811 pixels per millimeter)
  - Image Dimensions: 3508 x 4961 pixels

Unified 1:1 Components Included on the Same Sheet:
  1. Priority 1 - ArUco Calibration Markers (DICT_4X4_50, IDs 0, 1, 2, 3) at exact coordinates:
     - ArUco #0 (Rear-Left):   (X = 70.0 mm, Y = 180.0 mm)
     - ArUco #1 (Rear-Right):  (X = 330.0 mm, Y = 180.0 mm)
     - ArUco #2 (Front-Left):  (X = 70.0 mm, Y = 40.0 mm)
     - ArUco #3 (Front-Right): (X = 330.0 mm, Y = 40.0 mm)
     - Active Arena (260 x 140 mm): 100% spotless clean white inside!

  2. Priority 2 - xArm 1S Manipulator Mounting Section (X = 200.0 mm, Y = 280.0 mm):
     - Baseplate perimeter: 140.0 mm (W) x 154.0 mm (D)
     - Turret rotation circle: 96.0 mm diameter
     - 4x M4 clearance drill holes with center crosshairs:
       * Front-Left:  (X = 136.5 mm, Y = 208.5 mm)
       * Front-Right: (X = 263.5 mm, Y = 208.5 mm)
       * Rear-Left:   (X = 136.5 mm, Y = 351.5 mm)
       * Rear-Right:  (X = 263.5 mm, Y = 351.5 mm)
     - Heading vector (pointing South into picking arena)

  3. Priority 3 - Left Flank Collection Container Section:
     - Container pocket boundary & registration bracket: (X = 50.0 mm, Y = 280.0 mm)
     - 90° CW drop swing trajectory vector (r = 150.0 mm)

  4. Calibration Check Rulers (100.0 mm physical check) & Baseboard Alignment Guides.

Outputs:
  - docs/assets/aruco_markers/master_workcell_sheet_A3_1to1.png
  - docs/assets/aruco_markers/master_workcell_sheet_A3_1to1.pdf
"""

import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300.0
PPM = DPI / 25.4  # Pixels per mm (~11.811 px/mm)

PAGE_W_MM = 297.0
PAGE_H_MM = 420.0

PAGE_W_PX = int(round(PAGE_W_MM * PPM))  # 3508 px
PAGE_H_PX = int(round(PAGE_H_MM * PPM))  # 4961 px

def mm_to_px(mm: float) -> int:
    return int(round(mm * PPM))

def world_to_paper_mm(x_world: float, y_world: float):
    """
    Converts global baseboard world coordinates to A3 paper coordinates.
    Sheet is centered along X = 200.0 mm (X_paper_left = 51.5 mm, X_paper_right = 348.5 mm).
    Sheet bottom edge is at Y_world = 0.0 mm (Front of baseboard).
    """
    x_paper = x_world - 51.5
    y_paper = PAGE_H_MM - y_world  # top of page is Y_paper = 0
    return x_paper, y_paper

def world_to_paper_px(x_world: float, y_world: float):
    x_p, y_p = world_to_paper_mm(x_world, y_world)
    return mm_to_px(x_p), mm_to_px(y_p)


def generate_a3_master_sheet(output_dir: str = "docs/assets/aruco_markers"):
    os.makedirs(output_dir, exist_ok=True)

    # 1. Blank white 300 DPI canvas
    canvas = np.full((PAGE_H_PX, PAGE_W_PX, 3), 255, dtype=np.uint8)

    # 2. Setup ArUco Dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_size_mm = 26.0
    marker_size_px = mm_to_px(marker_size_mm)
    half_m_px = marker_size_px // 2

    markers_spec = [
        {"id": 0, "role": "Rear-Left",  "world_x": 70.0,  "world_y": 180.0},
        {"id": 1, "role": "Rear-Right", "world_x": 330.0, "world_y": 180.0},
        {"id": 2, "role": "Front-Left", "world_x": 70.0,  "world_y": 40.0},
        {"id": 3, "role": "Front-Right","world_x": 330.0, "world_y": 40.0},
    ]

    # --- Active Picking Arena Boundary (260.0 x 140.0 mm) ---
    p0_px = world_to_paper_px(70.0, 180.0)
    p3_px = world_to_paper_px(330.0, 40.0)
    # Subtle light outline for foam perimeter placement (Zero text or marks inside!)
    cv2.rectangle(canvas, p0_px, p3_px, (220, 225, 230), 1)

    # --- Render 4 ArUco Markers ---
    for spec in markers_spec:
        cx_px, cy_px = world_to_paper_px(spec["world_x"], spec["world_y"])
        m_img = cv2.aruco.generateImageMarker(aruco_dict, spec["id"], marker_size_px, borderBits=1)
        m_bgr = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)

        x1 = cx_px - half_m_px
        y1 = cy_px - half_m_px
        x2 = x1 + marker_size_px
        y2 = y1 + marker_size_px
        canvas[y1:y2, x1:x2] = m_bgr

        # Precision center crosshair ticks outside the black marker
        t_len = mm_to_px(3.0)
        cv2.line(canvas, (cx_px - half_m_px - t_len, cy_px), (cx_px - half_m_px, cy_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px + half_m_px, cy_px), (cx_px + half_m_px + t_len, cy_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px, cy_px - half_m_px - t_len), (cx_px, cy_px - half_m_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px, cy_px + half_m_px), (cx_px, cy_px + half_m_px + t_len), (0, 0, 220), 2)

    # --- Render xArm 1S Manipulator Baseplate Section (X = 200, Y = 280) ---
    bx, by = 200.0, 280.0
    bx_px, by_px = world_to_paper_px(bx, by)

    # Baseplate rectangle: 140 mm width x 154 mm length
    bp_tl = world_to_paper_px(bx - 70.0, by + 77.0)
    bp_br = world_to_paper_px(bx + 70.0, by - 77.0)
    cv2.rectangle(canvas, bp_tl, bp_br, (219, 234, 254), -1)  # Soft blue fill
    cv2.rectangle(canvas, bp_tl, bp_br, (37, 99, 235), 4)     # Blue border

    # Turret rotation circular ring (diameter 96 mm -> radius 48 mm)
    r_turret_px = mm_to_px(48.0)
    cv2.circle(canvas, (bx_px, by_px), r_turret_px, (59, 130, 246), 3)
    cv2.circle(canvas, (bx_px, by_px), mm_to_px(6.0), (29, 78, 216), -1)  # Center pivot hole

    # Heading vector (Facing South into picking arena)
    head_tip_px = world_to_paper_px(bx, by - 65.0)
    cv2.line(canvas, (bx_px, by_px), head_tip_px, (29, 78, 216), 5)
    # Arrowhead
    arrow_pts = np.array([
        [head_tip_px[0] - mm_to_px(4.0), head_tip_px[1] - mm_to_px(6.0)],
        [head_tip_px[0] + mm_to_px(4.0), head_tip_px[1] - mm_to_px(6.0)],
        [head_tip_px[0], head_tip_px[1]]
    ], dtype=np.int32)
    cv2.fillPoly(canvas, [arrow_pts], (29, 78, 216))

    # 4x M4 Mounting Screw Holes with Crosshairs
    # dx = +-63.5 mm (127 mm center-to-center), dy = +-71.5 mm (143 mm center-to-center)
    hole_offsets = [
        (-63.5, 71.5, "Rear-Left Hole (M4)"),
        (63.5, 71.5, "Rear-Right Hole (M4)"),
        (-63.5, -71.5, "Front-Left Hole (M4)"),
        (63.5, -71.5, "Front-Right Hole (M4)"),
    ]
    hole_radius_px = mm_to_px(2.25)  # 4.5 mm clearance drill hole

    for dx, dy, hname in hole_offsets:
        hx_w, hy_w = bx + dx, by + dy
        hx_px, hy_px = world_to_paper_px(hx_w, hy_w)
        # Clear white drill circle with dark outline
        cv2.circle(canvas, (hx_px, hy_px), hole_radius_px, (255, 255, 255), -1)
        cv2.circle(canvas, (hx_px, hy_px), hole_radius_px, (15, 23, 42), 2)
        # Drill center crosshair tick
        ch_len = mm_to_px(4.0)
        cv2.line(canvas, (hx_px - ch_len, hy_px), (hx_px + ch_len, hy_px), (220, 38, 38), 2)
        cv2.line(canvas, (hx_px, hy_px - ch_len), (hx_px, hy_px + ch_len), (220, 38, 38), 2)

    # --- Render Left Flank Collection Container Section (X = 50, Y = 280) ---
    # Container boundary: 100 mm x 100 mm -> X in [0, 100], Y in [230, 330]
    # On paper, X starts at 51.5 mm, so right half is X in [51.5, 100.0] mm
    box_r_top_px = world_to_paper_px(100.0, 330.0)
    box_r_bot_px = world_to_paper_px(51.5, 230.0)
    cv2.rectangle(canvas, (0, box_r_top_px[1]), (box_r_top_px[0], box_r_bot_px[1]), (250, 245, 255), -1)  # Soft purple fill
    cv2.rectangle(canvas, (0, box_r_top_px[1]), (box_r_top_px[0], box_r_bot_px[1]), (147, 51, 234), 3)

    # Swing arc / line from base center (200, 280) to left flank container
    p_drop_target_px = world_to_paper_px(51.5, 280.0)
    cv2.line(canvas, (bx_px, by_px), p_drop_target_px, (147, 51, 234), 4)

    # 3. Convert to PIL for sharp typography rendering
    pil_img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    ]
    try:
        font_main = ImageFont.truetype(font_paths[0], 26)
        font_bold = ImageFont.truetype(font_paths[1], 30)
        font_title = ImageFont.truetype(font_paths[1], 40)
        font_small = ImageFont.truetype(font_paths[0], 22)
        font_tiny = ImageFont.truetype(font_paths[0], 19)
        font_mono_bold = ImageFont.truetype(font_paths[2], 24)
        font_big_bold = ImageFont.truetype(font_paths[1], 34)
    except Exception:
        font_main = ImageFont.load_default()
        font_bold = font_main
        font_title = font_main
        font_small = font_main
        font_tiny = font_main
        font_mono_bold = font_main
        font_big_bold = font_main

    # --- TOP HEADER & CALIBRATION VERIFICATION (Y_paper in [5, 55 mm]) ---
    draw.text((mm_to_px(25.0), mm_to_px(8.0)), "Single xArm 1S - 1:1 True-Scale Master Workcell Sheet (A3)", fill=(15, 23, 42), font=font_title)
    draw.text((mm_to_px(25.0), mm_to_px(15.5)), "PRINT AT 100% SCALE (No Scaling / Actual Size) ON STANDARD A3 PAPER (297 x 420 mm)", fill=(185, 28, 28), font=font_bold)
    draw.text((mm_to_px(25.0), mm_to_px(21.5)), "Includes: 4x Coplanar ArUco Markers + xArm 1S Mount & Drill Holes + Flank Container Guide", fill=(71, 85, 105), font=font_main)

    # 100 mm Physical Scale Check Ruler
    ruler_x = mm_to_px(165.0)
    ruler_y = mm_to_px(31.0)
    ruler_w = mm_to_px(100.0)
    ruler_h = mm_to_px(7.0)
    draw.rectangle([ruler_x, ruler_y, ruler_x + ruler_w, ruler_y + ruler_h], outline=(15, 23, 42), width=2, fill=(245, 247, 250))
    draw.text((ruler_x, ruler_y - mm_to_px(5.5)), "PHYSICAL CHECK: This ruler bar must measure exactly 100.0 mm", fill=(15, 23, 42), font=font_bold)

    for mm in range(0, 101, 10):
        tx = ruler_x + mm_to_px(float(mm))
        th = mm_to_px(6.0) if mm % 50 == 0 else (mm_to_px(4.0) if mm % 10 == 0 else mm_to_px(2.5))
        draw.line([(tx, ruler_y), (tx, ruler_y + th)], fill=(15, 23, 42), width=2)
        if mm % 20 == 0:
            draw.text((tx - mm_to_px(2.5), ruler_y + mm_to_px(1.5)), f"{mm}", fill=(15, 23, 42), font=font_tiny)

    # --- xArm 1S MOUNTING CALLOUTS & HOLE LABELS (Y_paper in [63, 217 mm]) ---
    # Baseplate Header
    draw.text((mm_to_px(85.0), mm_to_px(72.0)), "xArm 1S Baseplate (140 x 154 mm)\nBase Center: (X = 200.0, Y = 280.0 mm)", fill=(30, 58, 138), font=font_big_bold, align="center")

    # Heading callout
    draw.text((mm_to_px(155.0), mm_to_px(162.0)), "Nominal Heading: -90.0°\n(Points South into Arena)", fill=(29, 78, 216), font=font_bold)

    # 4 Drill Hole Dimension Callouts
    # Hole Rear-Left (136.5, 351.5) -> (85.0 mm, 68.5 mm)
    draw.text((mm_to_px(42.0), mm_to_px(62.0)), "DRILL HOLE #1 (Rear-Left)\nØ 4.5 mm (M4 Screw)\n(136.5, 351.5 mm)", fill=(15, 23, 42), font=font_tiny, align="right")
    # Hole Rear-Right (263.5, 351.5) -> (212.0 mm, 68.5 mm)
    draw.text((mm_to_px(220.0), mm_to_px(62.0)), "DRILL HOLE #2 (Rear-Right)\nØ 4.5 mm (M4 Screw)\n(263.5, 351.5 mm)", fill=(15, 23, 42), font=font_tiny, align="left")
    # Hole Front-Left (136.5, 208.5) -> (85.0 mm, 211.5 mm)
    draw.text((mm_to_px(42.0), mm_to_px(204.0)), "DRILL HOLE #3 (Front-Left)\nØ 4.5 mm (M4 Screw)\n(136.5, 208.5 mm)", fill=(15, 23, 42), font=font_tiny, align="right")
    # Hole Front-Right (263.5, 208.5) -> (212.0 mm, 211.5 mm)
    draw.text((mm_to_px(220.0), mm_to_px(204.0)), "DRILL HOLE #4 (Front-Right)\nØ 4.5 mm (M4 Screw)\n(263.5, 208.5 mm)", fill=(15, 23, 42), font=font_tiny, align="left")

    # Center-to-center hole spacing callouts
    draw.text((mm_to_px(118.0), mm_to_px(105.0)), "◄ 127.0 mm Width ►", fill=(71, 85, 105), font=font_bold)
    draw.text((mm_to_px(223.0), mm_to_px(135.0)), "▲\n143.0 mm\nLength\n▼", fill=(71, 85, 105), font=font_bold, align="center")

    # --- LEFT FLANK CONTAINER CALLOUTS (Y_paper in [90, 190 mm]) ---
    draw.text((mm_to_px(5.0), mm_to_px(95.0)),
              "◄ LEFT FLANK CONTAINER POCKET (100x100 mm)\nDrop Target: (X = 50.0, Y = 280.0 mm)\n90° CW Swing | Radial Reach: r = 150.0 mm",
              fill=(126, 34, 206), font=font_bold)

    # --- BUFFER ZONE CALLOUT (Y_world in [180, 203 mm] -> Y_paper in [217, 240 mm]) ---
    draw.text((mm_to_px(85.0), mm_to_px(225.0)),
              "=== +23.0 mm Physical Buffer Zone between xArm Baseplate and Arena Rear ===",
              fill=(100, 116, 139), font=font_bold)

    # --- ARUCO MARKER LABELS (SAFELY CLEAR OF MARKERS AND OUTSIDE THE ARENA) ---
    # Left markers (0 and 2) are at X_paper = 18.5 mm (occupy X in [5.5, 31.5] mm).
    # Labels are placed at X_paper = 40.0 mm with '◄' pointing to the marker.
    # Right markers (1 and 3) are at X_paper = 278.5 mm (occupy X in [265.5, 291.5] mm).
    # Labels are placed at X_paper = 158.0 mm with '►' pointing to the marker.

    # ArUco #0: Rear-Left (70.0, 180.0 mm)
    draw.text((mm_to_px(40.0), mm_to_px(224.0)),
              "◄ ArUco #0 [Rear-Left Corner]\n  Center: (X = 70.0, Y = 180.0 mm)",
              fill=(15, 23, 42), font=font_small)

    # ArUco #1: Rear-Right (330.0, 180.0 mm)
    draw.text((mm_to_px(158.0), mm_to_px(224.0)),
              "ArUco #1 [Rear-Right Corner] ►\nCenter: (X = 330.0, Y = 180.0 mm)",
              fill=(15, 23, 42), font=font_small)

    # ArUco #2: Front-Left (70.0, 40.0 mm) -> Arena Origin Datum!
    draw.text((mm_to_px(40.0), mm_to_px(390.0)),
              "◄ ★ ArUco #2 [ARENA ORIGIN (Front-Left)]\n"
              "  World: (X = 70.0, Y = 40.0 mm) | Offset: +18.5mm Left, +40mm Bottom",
              fill=(185, 28, 28), font=font_small)

    # ArUco #3: Front-Right (330.0, 40.0 mm)
    draw.text((mm_to_px(158.0), mm_to_px(390.0)),
              "ArUco #3 [Front-Right Corner] ►\nCenter: (X = 330.0, Y = 40.0 mm)",
              fill=(15, 23, 42), font=font_small)

    # --- CENTERLINE ALIGNMENT TICKS (X_world = 200.0 mm -> X_paper = 148.5 mm) ---
    cl_x_px = mm_to_px(148.5)
    # Top edge centerline tick
    draw.line([(cl_x_px, 0), (cl_x_px, mm_to_px(6.0))], fill=(37, 99, 235), width=3)
    draw.text((cl_x_px - mm_to_px(28.0), mm_to_px(1.5)), "▼ CENTERLINE (X = 200.0 mm) ▼", fill=(37, 99, 235), font=font_tiny)

    # Buffer zone centerline tick (between arena rear Y=180 and xArm base front Y=203)
    buf_top_px = mm_to_px(PAGE_H_MM - 203.0)
    buf_bot_px = mm_to_px(PAGE_H_MM - 180.0)
    draw.line([(cl_x_px, buf_top_px), (cl_x_px, buf_bot_px)], fill=(37, 99, 235), width=2)

    # Bottom edge centerline tick
    draw.line([(cl_x_px, mm_to_px(PAGE_H_MM - 6.0)), (cl_x_px, PAGE_H_PX)], fill=(37, 99, 235), width=3)
    draw.text((cl_x_px - mm_to_px(28.0), mm_to_px(PAGE_H_MM - 5.5)), "▲ CENTERLINE (X = 200.0 mm) ▲", fill=(37, 99, 235), font=font_tiny)

    # --- BOTTOM BASEBOARD ALIGNMENT GUIDELINE (Y_paper in [410, 418 mm]) ---
    b_y = mm_to_px(PAGE_H_MM - 2.5)
    draw.line([(0, b_y), (PAGE_W_PX, b_y)], fill=(220, 38, 38), width=4)
    draw.text((mm_to_px(35.0), mm_to_px(PAGE_H_MM - 8.0)),
              "▲ ALIGN BOTTOM SHEET EDGE FLUSH WITH FRONT OF WORKBENCH / BOARD (Y = 0.0 mm) ▲",
              fill=(220, 38, 38), font=font_bold)

    # Global Origin (0,0) Left Offset Indicator
    draw.text((mm_to_px(3.0), mm_to_px(PAGE_H_MM - 14.0)),
              "◄--- 51.5 mm to Global Origin (0, 0) ---| [Left Paper Edge: X = 51.5 mm]",
              fill=(30, 58, 138), font=font_mono_bold)

    # 4. FINAL PASS: Ensure ArUco markers are 100% pristine with ZERO text or line contamination
    final_canvas = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    for spec in markers_spec:
        cx_px, cy_px = world_to_paper_px(spec["world_x"], spec["world_y"])
        # Blank quiet zone (2mm beyond marker edge)
        qz = half_m_px + mm_to_px(2.0)
        cv2.rectangle(final_canvas, (cx_px - qz, cy_px - qz), (cx_px + qz, cy_px + qz), (255, 255, 255), -1)

        # Draw pristine ArUco marker
        m_img = cv2.aruco.generateImageMarker(aruco_dict, spec["id"], marker_size_px, borderBits=1)
        m_bgr = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)
        x1 = cx_px - half_m_px
        y1 = cy_px - half_m_px
        final_canvas[y1:y1 + marker_size_px, x1:x1 + marker_size_px] = m_bgr

        # Draw red center crosshair ticks strictly outside the black marker (with 0.5mm clear gap)
        t_gap = mm_to_px(0.5)
        t_len = mm_to_px(3.5)
        cv2.line(final_canvas, (cx_px - half_m_px - t_len, cy_px), (cx_px - half_m_px - t_gap, cy_px), (0, 0, 220), 2)
        cv2.line(final_canvas, (cx_px + half_m_px + t_gap, cy_px), (cx_px + half_m_px + t_len, cy_px), (0, 0, 220), 2)
        cv2.line(final_canvas, (cx_px, cy_px - half_m_px - t_len), (cx_px, cy_px - half_m_px - t_gap), (0, 0, 220), 2)
        cv2.line(final_canvas, (cx_px, cy_px + half_m_px + t_gap), (cx_px, cy_px + half_m_px + t_len), (0, 0, 220), 2)

    pil_img = Image.fromarray(cv2.cvtColor(final_canvas, cv2.COLOR_BGR2RGB))

    # Save outputs
    png_path = os.path.join(output_dir, "master_workcell_sheet_A3_1to1.png")
    pdf_path = os.path.join(output_dir, "master_workcell_sheet_A3_1to1.pdf")

    pil_img.save(png_path, "PNG", dpi=(300, 300))
    pil_img.save(pdf_path, "PDF", resolution=300.0)

    print(f"Successfully generated 1:1 true-scale A3 master workcell sheets:")
    print(f"  PNG (300 DPI): {png_path} ({PAGE_W_PX}x{PAGE_H_PX} px)")
    print(f"  PDF (A3 1:1):  {pdf_path}")
    return png_path, pdf_path


if __name__ == "__main__":
    generate_a3_master_sheet()
