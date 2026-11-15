#!/usr/bin/env python3
"""
Generate printable ArUco markers (DICT_4X4_50, IDs 0-3) for arena corner calibration.
Outputs high-resolution individual PNGs and a ready-to-print sheet.
"""

import os
import cv2
import numpy as np

def generate_markers(output_dir: str = "docs/assets/aruco_markers"):
    os.makedirs(output_dir, exist_ok=True)
    
    # DICT_4X4_50 dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    markers_info = [
        {"id": 0, "name": "marker_0_rear_left.png", "label": "ID 0: Rear-Left (85mm, 255mm)"},
        {"id": 1, "name": "marker_1_rear_right.png", "label": "ID 1: Rear-Right (475mm, 255mm)"},
        {"id": 2, "name": "marker_2_front_left.png", "label": "ID 2: Front-Left (85mm, 45mm)"},
        {"id": 3, "name": "marker_3_front_right.png", "label": "ID 3: Front-Right (475mm, 45mm)"},
    ]
    
    marker_px = 600  # High-res 600x600 px
    border_px = 60   # White quiet-zone margin around black border
    
    saved_paths = []
    
    for info in markers_info:
        # Generate marker with 1 bit border
        img = cv2.aruco.generateImageMarker(aruco_dict, info["id"], marker_px, borderBits=1)
        
        # Add white quiet zone around marker
        padded = np.full((marker_px + 2 * border_px, marker_px + 2 * border_px), 255, dtype=np.uint8)
        padded[border_px:border_px + marker_px, border_px:border_px + marker_px] = img
        
        # Convert to BGR to add text
        bgr = cv2.cvtColor(padded, cv2.COLOR_GRAY2BGR)
        cv2.putText(bgr, info["label"], (border_px, border_px // 2 + 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2, cv2.LINE_AA)
        
        out_path = os.path.join(output_dir, info["name"])
        cv2.imwrite(out_path, bgr)
        saved_paths.append(out_path)
        print(f"Generated: {out_path}")

    # Generate a combined A4 printable sheet (300 DPI, A4 is 2480 x 3508 px)
    # 30 mm at 300 DPI is ~354 px
    sheet = np.full((3508, 2480, 3), 255, dtype=np.uint8)
    
    title = "Single xArm 1s - Arena ArUco Calibration Markers (DICT_4X4_50)"
    cv2.putText(sheet, title, (120, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 3, cv2.LINE_AA)
    
    instructions = [
        "Instructions: Print at 100% scale (No scaling / Actual Size) on A4 matte paper or sticker sheet.",
        "Each marker outer black boundary should measure exactly 30 mm x 30 mm (or up to 40 mm).",
        "Cut along the light grey dotted outlines and affix flat directly on the MDF board at Z = 0.0 mm."
    ]
    for i, line in enumerate(instructions):
        cv2.putText(sheet, line, (120, 270 + i * 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2, cv2.LINE_AA)
        
    # Draw a 50 mm calibration check ruler at top right
    # 50 mm @ 300 DPI = 590 px (1 mm = 11.81 px)
    ruler_x, ruler_y = 1700, 200
    ruler_w = int(50 * 11.811)
    cv2.rectangle(sheet, (ruler_x, ruler_y), (ruler_x + ruler_w, ruler_y + 40), (0, 0, 0), 2)
    cv2.putText(sheet, "50 mm Check Ruler", (ruler_x, ruler_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
    for mm in range(0, 51, 10):
        tick_x = ruler_x + int(mm * 11.811)
        cv2.line(sheet, (tick_x, ruler_y), (tick_x, ruler_y + 20), (0, 0, 0), 2)
        cv2.putText(sheet, f"{mm}", (tick_x - 10, ruler_y + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Place 4 markers on the sheet in a 2x2 grid
    marker_print_px = int(35 * 11.811) # 35 mm marker size
    positions = [
        # (col, row)
        (300, 600, markers_info[0]),   # ID 0
        (1300, 600, markers_info[1]),  # ID 1
        (300, 1600, markers_info[2]),  # ID 2
        (1300, 1600, markers_info[3])  # ID 3
    ]
    
    for px, py, info in positions:
        marker_raw = cv2.aruco.generateImageMarker(aruco_dict, info["id"], marker_print_px, borderBits=1)
        marker_bgr = cv2.cvtColor(marker_raw, cv2.COLOR_GRAY2BGR)
        
        # Draw light gray border for cutting
        cut_margin = 40
        cv2.rectangle(sheet, (px - cut_margin, py - cut_margin), 
                      (px + marker_print_px + cut_margin, py + marker_print_px + cut_margin), 
                      (200, 200, 200), 2)
        
        # Insert marker
        sheet[py:py + marker_print_px, px:px + marker_print_px] = marker_bgr
        
        # Text description
        cv2.putText(sheet, info["label"], (px - cut_margin, py + marker_print_px + 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(sheet, f"Dictionary: DICT_4X4_50 | Marker ID: {info['id']}", 
                    (px - cut_margin, py + marker_print_px + 120), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2, cv2.LINE_AA)

    sheet_path = os.path.join(output_dir, "aruco_markers_printable_sheet.png")
    cv2.imwrite(sheet_path, sheet)
    print(f"Generated printable sheet: {sheet_path}")
    saved_paths.append(sheet_path)
    return saved_paths

if __name__ == "__main__":
    generate_markers()
