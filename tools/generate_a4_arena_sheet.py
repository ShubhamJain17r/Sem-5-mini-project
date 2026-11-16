#!/usr/bin/env python3
"""
Generate a 1:1 True-Scale A4 Printable Arena Sheet for Single xArm 1S Workcell.

Dimensions & Scale:
  - Standard A4 Landscape: 297.0 mm x 210.0 mm
  - Print Resolution: 300 DPI (11.811 pixels per millimeter)
  - Image Dimensions: 3508 x 2480 pixels

Layout & Positions (1:1 Metric Placement on Baseboard):
  - Baseboard dimensions: 400.0 mm (W) x 400.0 mm (D)
  - Sheet horizontal alignment: Centered at X = 200.0 mm (X in [51.5 mm, 348.5 mm])
  - Sheet vertical alignment: Bottom edge at Front of Board (Y = 0.0 mm, Y in [0.0 mm, 210.0 mm])
  - ArUco #0 (Rear-Left):   Center at (X = 70.0 mm, Y = 180.0 mm) -> Paper (X = 18.5 mm, Y = 30.0 mm from top)
  - ArUco #1 (Rear-Right):  Center at (X = 330.0 mm, Y = 180.0 mm) -> Paper (X = 278.5 mm, Y = 30.0 mm from top)
  - ArUco #2 (Front-Left):  Center at (X = 70.0 mm, Y = 40.0 mm) -> Paper (X = 18.5 mm, Y = 170.0 mm from top)
  - ArUco #3 (Front-Right): Center at (X = 330.0 mm, Y = 40.0 mm) -> Paper (X = 278.5 mm, Y = 170.0 mm from top)

Outputs:
  - docs/assets/aruco_markers/arena_aruco_sheet_A4_1to1.png
  - docs/assets/aruco_markers/arena_aruco_sheet_A4_1to1.pdf
"""

import os
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DPI = 300.0
PPM = DPI / 25.4  # Pixels per mm (~11.811 px/mm)

PAGE_W_MM = 297.0
PAGE_H_MM = 210.0

PAGE_W_PX = int(round(PAGE_W_MM * PPM))  # 3508 px
PAGE_H_PX = int(round(PAGE_H_MM * PPM))  # 2480 px

def mm_to_px(mm: float) -> int:
    return int(round(mm * PPM))

def world_to_paper_mm(x_world: float, y_world: float):
    """
    Converts global baseboard world mm to A4 paper mm.
    Paper Left edge is at X_world = 51.5 mm.
    Paper Bottom edge is at Y_world = 0.0 mm (Y_paper = 210.0 mm from top).
    """
    x_paper = x_world - 51.5
    y_paper = PAGE_H_MM - y_world  # top of page is Y_paper = 0
    return x_paper, y_paper

def world_to_paper_px(x_world: float, y_world: float):
    x_p, y_p = world_to_paper_mm(x_world, y_world)
    return mm_to_px(x_p), mm_to_px(y_p)


def generate_arena_sheet(output_dir: str = "docs/assets/aruco_markers"):
    os.makedirs(output_dir, exist_ok=True)

    # 1. Create blank white 300 DPI canvas
    canvas = np.full((PAGE_H_PX, PAGE_W_PX, 3), 255, dtype=np.uint8)

    # 2. Setup ArUco Dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Marker configuration: 26.0 mm size
    marker_size_mm = 26.0
    marker_size_px = mm_to_px(marker_size_mm)

    markers_spec = [
        {"id": 0, "role": "Rear-Left",  "world_x": 70.0,  "world_y": 180.0},
        {"id": 1, "role": "Rear-Right", "world_x": 330.0, "world_y": 180.0},
        {"id": 2, "role": "Front-Left", "world_x": 70.0,  "world_y": 40.0},
        {"id": 3, "role": "Front-Right","world_x": 330.0, "world_y": 40.0},
    ]

    # Draw active picking arena outline (260.0 mm x 140.0 mm between marker centers)
    p0_px = world_to_paper_px(70.0, 180.0)
    p1_px = world_to_paper_px(330.0, 180.0)
    p2_px = world_to_paper_px(70.0, 40.0)
    p3_px = world_to_paper_px(330.0, 40.0)

    # Clean, subtle guideline rectangle between marker centers for perimeter tape placement
    cv2.rectangle(canvas, p0_px, p3_px, (220, 225, 230), 1)

    # 3. Render ArUco markers at exact millimeter locations
    half_m_px = marker_size_px // 2

    for spec in markers_spec:
        cx_px, cy_px = world_to_paper_px(spec["world_x"], spec["world_y"])
        
        # Generate marker with 1-bit border
        m_img = cv2.aruco.generateImageMarker(aruco_dict, spec["id"], marker_size_px, borderBits=1)
        m_bgr = cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR)

        # Place on canvas
        x1 = cx_px - half_m_px
        y1 = cy_px - half_m_px
        x2 = x1 + marker_size_px
        y2 = y1 + marker_size_px
        canvas[y1:y2, x1:x2] = m_bgr

        # Draw red center crosshair tick (outside the black marker to maintain detection)
        tick_len = mm_to_px(3.0)
        cv2.line(canvas, (cx_px - half_m_px - tick_len, cy_px), (cx_px - half_m_px, cy_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px + half_m_px, cy_px), (cx_px + half_m_px + tick_len, cy_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px, cy_px - half_m_px - tick_len), (cx_px, cy_px - half_m_px), (0, 0, 220), 2)
        cv2.line(canvas, (cx_px, cy_px + half_m_px), (cx_px, cy_px + half_m_px + tick_len), (0, 0, 220), 2)

    # 4. Convert to PIL for sharp typography rendering (OUTSIDE ARENA ONLY)
    pil_img = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    # Load system TrueType fonts
    font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    ]
    try:
        font_main = ImageFont.truetype(font_paths[0], 24)
        font_bold = ImageFont.truetype(font_paths[1], 28)
        font_title = ImageFont.truetype(font_paths[1], 36)
        font_small = ImageFont.truetype(font_paths[0], 20)
        font_mono_bold = ImageFont.truetype(font_paths[2], 22)
    except Exception:
        font_main = ImageFont.load_default()
        font_bold = font_main
        font_title = font_main
        font_small = font_main
        font_mono_bold = font_main

    # --- TOP MARGIN (Y_paper < 30.0 mm): Title, xArm trim line, Rear Marker Labels ---
    # Title & scale notes
    draw.text((mm_to_px(40.0), mm_to_px(4.0)), "Single xArm 1S - 1:1 True-Scale Picking Arena Sheet", fill=(15, 23, 42), font=font_title)
    draw.text((mm_to_px(40.0), mm_to_px(10.5)), "PRINT AT 100% SCALE (No Scaling / Actual Size) ON STANDARD A4 PAPER", fill=(185, 28, 28), font=font_bold)

    # xArm Baseplate Clearance / Trim Line at Y_world = 200 mm (Y_paper = 10.0 mm)
    trim_y = mm_to_px(PAGE_H_MM - 200.0)
    for tx in range(0, PAGE_W_PX, mm_to_px(6.0)):
        draw.line([(tx, trim_y), (tx + mm_to_px(3.5), trim_y)], fill=(180, 180, 180), width=2)
    draw.text((mm_to_px(40.0), trim_y + mm_to_px(1.5)),
              "✂ xArm CLEARANCE TRIM LINE (Y = 200.0 mm): Trim along dashed line if xArm base is already screwed at Y = 203 mm",
              fill=(120, 120, 120), font=font_small)

    # Marker labels (safely placed to the side with arrows pointing to markers)
    # Marker 0: Rear-Left (70.0, 180.0 mm) -> X_paper = 18.5 mm, Y_paper = 30.0 mm
    draw.text((mm_to_px(38.0), mm_to_px(16.0)), "◄ ArUco #0 [Rear-Left] (70.0, 180.0 mm)", fill=(15, 23, 42), font=font_small)

    # Marker 1: Rear-Right (330.0, 180.0 mm) -> X_paper = 278.5 mm, Y_paper = 30.0 mm
    draw.text((mm_to_px(160.0), mm_to_px(16.0)), "ArUco #1 [Rear-Right] (330.0, 180.0 mm) ►", fill=(15, 23, 42), font=font_small)

    # --- BOTTOM MARGIN (Y_paper > 170.0 mm): Calibration Ruler, Front Marker Labels, Alignment Guides ---
    # Marker 2: Front-Left (70.0, 40.0 mm) -> X_paper = 18.5 mm, Y_paper = 170.0 mm
    draw.text((mm_to_px(38.0), mm_to_px(184.0)), "◄ ArUco #2 [Front-Left: ARENA ORIGIN] (70.0, 40.0 mm)", fill=(185, 28, 28), font=font_small)

    # Marker 3: Front-Right (330.0, 40.0 mm) -> X_paper = 278.5 mm, Y_paper = 170.0 mm
    draw.text((mm_to_px(160.0), mm_to_px(184.0)), "ArUco #3 [Front-Right] (330.0, 40.0 mm) ►", fill=(15, 23, 42), font=font_small)

    # Calibration Check Ruler (100.0 mm physical check) centered at Y_paper = 175.0 mm
    ruler_x = mm_to_px(98.5)
    ruler_y = mm_to_px(175.0)
    ruler_w = mm_to_px(100.0)
    ruler_h = mm_to_px(6.0)
    draw.rectangle([ruler_x, ruler_y, ruler_x + ruler_w, ruler_y + ruler_h], outline=(15, 23, 42), width=2, fill=(245, 247, 250))
    draw.text((ruler_x, ruler_y - mm_to_px(4.5)), "PHYSICAL CHECK: This ruler bar must measure exactly 100.0 mm", fill=(15, 23, 42), font=font_small)

    for mm in range(0, 101, 10):
        tx = ruler_x + mm_to_px(float(mm))
        th = mm_to_px(5.0) if mm % 50 == 0 else (mm_to_px(3.5) if mm % 10 == 0 else mm_to_px(2.0))
        draw.line([(tx, ruler_y), (tx, ruler_y + th)], fill=(15, 23, 42), width=1)
        if mm % 20 == 0:
            draw.text((tx - mm_to_px(2.0), ruler_y + mm_to_px(1.5)), f"{mm}", fill=(15, 23, 42), font=font_small)

    # Baseboard Front Edge Flush Alignment Line (Bottom edge: Y_paper = 208 mm)
    b_y = mm_to_px(PAGE_H_MM - 2.0)
    draw.line([(0, b_y), (PAGE_W_PX, b_y)], fill=(220, 38, 38), width=3)
    draw.text((mm_to_px(15.0), mm_to_px(PAGE_H_MM - 6.5)),
              "▲ ALIGN BOTTOM PAPER EDGE FLUSH WITH FRONT OF WOODEN BASEBOARD (Y = 0.0 mm) ▲",
              fill=(220, 38, 38), font=font_bold)

    # Baseboard Left Margin Guide (Left edge: X_paper = 2 mm)
    l_x = mm_to_px(2.0)
    draw.line([(l_x, 0), (l_x, PAGE_H_PX)], fill=(37, 99, 235), width=3)
    draw.text((mm_to_px(4.0), mm_to_px(195.0)),
              "► Position left edge exactly 51.5 mm from board left edge (Centers arena at X = 200 mm) ►",
              fill=(37, 99, 235), font=font_small)

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
    png_path = os.path.join(output_dir, "arena_aruco_sheet_A4_1to1.png")
    pdf_path = os.path.join(output_dir, "arena_aruco_sheet_A4_1to1.pdf")

    pil_img.save(png_path, "PNG", dpi=(300, 300))
    pil_img.save(pdf_path, "PDF", resolution=300.0)

    print(f"Successfully generated 1:1 true-scale arena sheets:")
    print(f"  PNG (300 DPI): {png_path} ({PAGE_W_PX}x{PAGE_H_PX} px)")
    print(f"  PDF (A4 1:1):  {pdf_path}")
    return png_path, pdf_path


if __name__ == "__main__":
    generate_arena_sheet()
