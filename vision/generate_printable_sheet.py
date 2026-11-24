#!/usr/bin/env python3
"""
generate_printable_sheet.py
----------------------------
Generates professional, printable ISO A4 size sheets containing 50x50 mm (5x5 cm)
ArUco calibration markers for the xArm 1S Vision Sorting Arena.

Strict Requirements Fulfilled:
- Exact 50.0 mm x 50.0 mm physical marker size when printed at 100% scale.
- Dictionary: cv2.aruco.DICT_4X4_50 (IDs 0, 1, 2, 3).
- Strictly NO text overlapping with markers or quiet zones (guaranteed clean cutouts).
- 5 mm white quiet zone with dashed cutting guide (60x60 mm cut box).
- 50x50 mm corner crop marks for flush cuts.
- 100 mm & 50 mm physical calibration check rulers to verify printer 1:1 scale.
- Arena installation mini-map showing exact mounting positions on the MDF board.
- Outputs multi-page vector PDF, single-page vector PDF, 300 DPI printable PNGs,
  individual marker PNGs, and a browser-printable HTML file.
"""

import os
import cv2
import numpy as np
import weasyprint

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
OUTPUT_DIR = os.path.join(REPO_ROOT, "docs/assets/aruco_markers")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ArUco Dictionary
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

MARKER_METADATA = {
    0: {
        "role": "Top-Left",
        "cell": "Row 3, Col 0",
        "world": "(X = 25.0 mm, Y = 425.0 mm, Z = 0.0 mm)",
        "warp_pixel": "(0, 0)",
        "notes": "Affix at rear-left corner cell, adjacent to Red Bin flank"
    },
    1: {
        "role": "Top-Right",
        "cell": "Row 3, Col 6",
        "world": "(X = 325.0 mm, Y = 425.0 mm, Z = 0.0 mm)",
        "warp_pixel": "(600, 0)",
        "notes": "Affix at rear-right corner cell, adjacent to Green Bin flank"
    },
    2: {
        "role": "Bottom-Left",
        "cell": "Row 11, Col 0",
        "world": "(X = 25.0 mm, Y = 25.0 mm, Z = 0.0 mm)",
        "warp_pixel": "(0, 800)",
        "notes": "Affix at front-left arena corner cell (near user edge)"
    },
    3: {
        "role": "Bottom-Right",
        "cell": "Row 11, Col 6",
        "world": "(X = 325.0 mm, Y = 25.0 mm, Z = 0.0 mm)",
        "warp_pixel": "(600, 800)",
        "notes": "Affix at front-right arena corner cell (near user edge)"
    }
}

def generate_marker_svg(marker_id: int, marker_size_mm: float = 50.0, quiet_zone_mm: float = 5.0) -> str:
    """
    Renders pure vector SVG for an ArUco marker from DICT_4X4_50.
    Total box = (marker_size + 2*quiet_zone) mm = 60x60 mm.
    Inner black marker = 50x50 mm.
    ZERO text inside the cut box.
    """
    grid = cv2.aruco.generateImageMarker(ARUCO_DICT, marker_id, 6, borderBits=1)
    bit_size = marker_size_mm / 6.0
    total_size = marker_size_mm + 2.0 * quiet_zone_mm
    mx = quiet_zone_mm
    my = quiet_zone_mm

    svg_parts = []
    svg_parts.append(f'<svg class="marker-svg" width="{total_size}mm" height="{total_size}mm" viewBox="0 0 {total_size} {total_size}" xmlns="http://www.w3.org/2000/svg">')
    
    # 1. White quiet zone background
    svg_parts.append(f'  <rect x="0" y="0" width="{total_size}" height="{total_size}" fill="#FFFFFF" />')
    
    # 2. Outer cut guide line (dashed light gray)
    svg_parts.append(
        f'  <rect x="0.25" y="0.25" width="{total_size-0.5:.2f}" height="{total_size-0.5:.2f}" '
        f'fill="none" stroke="#A0A0A0" stroke-width="0.5" stroke-dasharray="2,2" />'
    )
    
    # 3. Corner crop marks for 50x50 mm flush cut (L-shaped tick marks with 1.2mm gap)
    # The gap ensures the tick lines DO NOT touch the black marker quadrangle,
    # preventing any distortion in computer vision corner refinement.
    gap = 1.2 # mm offset away from marker edge
    cl = 2.8  # tick length in mm
    
    # Top-Left crop
    svg_parts.append(f'  <line x1="{mx-gap-cl:.2f}" y1="{my:.2f}" x2="{mx-gap:.2f}" y2="{my:.2f}" stroke="#777777" stroke-width="0.4" />')
    svg_parts.append(f'  <line x1="{mx:.2f}" y1="{my-gap-cl:.2f}" x2="{mx:.2f}" y2="{my-gap:.2f}" stroke="#777777" stroke-width="0.4" />')
    # Top-Right crop
    svg_parts.append(f'  <line x1="{mx+marker_size_mm+gap:.2f}" y1="{my:.2f}" x2="{mx+marker_size_mm+gap+cl:.2f}" y2="{my:.2f}" stroke="#777777" stroke-width="0.4" />')
    svg_parts.append(f'  <line x1="{mx+marker_size_mm:.2f}" y1="{my-gap-cl:.2f}" x2="{mx+marker_size_mm:.2f}" y2="{my-gap:.2f}" stroke="#777777" stroke-width="0.4" />')
    # Bottom-Left crop
    svg_parts.append(f'  <line x1="{mx-gap-cl:.2f}" y1="{my+marker_size_mm:.2f}" x2="{mx-gap:.2f}" y2="{my+marker_size_mm:.2f}" stroke="#777777" stroke-width="0.4" />')
    svg_parts.append(f'  <line x1="{mx:.2f}" y1="{my+marker_size_mm+gap:.2f}" x2="{mx:.2f}" y2="{my+marker_size_mm+gap+cl:.2f}" stroke="#777777" stroke-width="0.4" />')
    # Bottom-Right crop
    svg_parts.append(f'  <line x1="{mx+marker_size_mm+gap:.2f}" y1="{my+marker_size_mm:.2f}" x2="{mx+marker_size_mm+gap+cl:.2f}" y2="{my+marker_size_mm:.2f}" stroke="#777777" stroke-width="0.4" />')
    svg_parts.append(f'  <line x1="{mx+marker_size_mm:.2f}" y1="{my+marker_size_mm+gap:.2f}" x2="{mx+marker_size_mm:.2f}" y2="{my+marker_size_mm+gap+cl:.2f}" stroke="#777777" stroke-width="0.4" />')
    
    # 4. Black marker quadrangle (50x50 mm)
    svg_parts.append(
        f'  <rect x="{mx:.4f}" y="{my:.4f}" width="{marker_size_mm:.4f}" height="{marker_size_mm:.4f}" '
        f'fill="#000000" shape-rendering="crispEdges" />'
    )
    
    # 5. White data bits
    for r in range(6):
        for c in range(6):
            if grid[r, c] == 255:
                bx = mx + c * bit_size
                by = my + r * bit_size
                svg_parts.append(
                    f'  <rect x="{bx:.4f}" y="{by:.4f}" width="{bit_size:.4f}" height="{bit_size:.4f}" '
                    f'fill="#FFFFFF" shape-rendering="crispEdges" />'
                )
                
    svg_parts.append('</svg>')
    return '\n'.join(svg_parts)


def build_ruler_svg(length_mm: float = 100.0) -> str:
    """Creates a precision 100 mm calibration check ruler SVG with 1 mm tick marks."""
    h_mm = 12.0
    w_mm = length_mm + 10.0
    svg = []
    svg.append(f'<svg width="{w_mm}mm" height="{h_mm}mm" viewBox="0 0 {w_mm} {h_mm}" xmlns="http://www.w3.org/2000/svg">')
    
    # Background strip
    ox = 5.0
    oy = 2.0
    svg.append(f'  <rect x="{ox}" y="{oy}" width="{length_mm}" height="7.0" fill="#F8F9FA" stroke="#222222" stroke-width="0.6"/>')
    
    # Highlight 0 to 50 mm (marker size)
    svg.append(f'  <rect x="{ox}" y="{oy}" width="50.0" height="7.0" fill="#E8F4FD" stroke="none"/>')
    
    for mm in range(int(length_mm) + 1):
        x = ox + mm
        if mm % 10 == 0:
            tick_h = 6.0
            svg.append(f'  <line x1="{x:.2f}" y1="{oy}" x2="{x:.2f}" y2="{oy + tick_h}" stroke="#111111" stroke-width="0.5"/>')
            svg.append(f'  <text x="{x:.2f}" y="{oy + 10.0}" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.6" font-weight="bold" text-anchor="middle" fill="#111111">{mm}</text>')
        elif mm % 5 == 0:
            tick_h = 4.0
            svg.append(f'  <line x1="{x:.2f}" y1="{oy}" x2="{x:.2f}" y2="{oy + tick_h}" stroke="#333333" stroke-width="0.4"/>')
        else:
            tick_h = 2.2
            svg.append(f'  <line x1="{x:.2f}" y1="{oy}" x2="{x:.2f}" y2="{oy + tick_h}" stroke="#666666" stroke-width="0.25"/>')
            
    # Text annotation
    svg.append(f'  <text x="{ox + 25.0}" y="{oy - 0.7}" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.2" font-weight="bold" text-anchor="middle" fill="#0055AA">◄── 50 mm (5.0 cm) ArUco Target Size ──►</text>')
    svg.append(f'  <text x="{ox + 75.0}" y="{oy - 0.7}" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.2" text-anchor="middle" fill="#555555">100 mm Verification Scale</text>')
    
    svg.append('</svg>')
    return '\n'.join(svg)


def build_arena_map_svg() -> str:
    """Renders a schematic diagram of the 350x600 mm arena showing ArUco positions."""
    w, h = 90.0, 54.0
    svg = []
    svg.append(f'<svg width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg">')
    
    # Outer arena baseplate (scale: 1 mm in arena = 0.12 mm in SVG)
    # Arena 350w x 600h mm -> 42 mm x 72 mm, let's fit comfortably in 84x48
    sx, sy = 0.22, 0.082
    
    # Arena border
    svg.append(f'  <rect x="4" y="2" width="78" height="50" rx="2" fill="#FAFAFA" stroke="#444444" stroke-width="0.7"/>')
    svg.append(f'  <text x="43" y="5.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.6" font-weight="bold" text-anchor="middle" fill="#222222">xArm 1S Arena Top-Down Layout (350 × 600 mm)</text>')
    
    # Top flank containers (Rows 0-2)
    svg.append(f'  <rect x="7" y="8" width="18" height="9" fill="#FFDADA" stroke="#CC0000" stroke-width="0.5"/>')
    svg.append(f'  <text x="16" y="13.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" text-anchor="middle" fill="#990000">Red Bin</text>')
    
    svg.append(f'  <rect x="61" y="8" width="18" height="9" fill="#DAF5DA" stroke="#008800" stroke-width="0.5"/>')
    svg.append(f'  <text x="70" y="13.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" text-anchor="middle" fill="#006600">Green Bin</text>')
    
    # xArm mount (Row 4, Col 3)
    svg.append(f'  <rect x="34" y="13" width="18" height="12" rx="1.5" fill="#E6EEF8" stroke="#1A569A" stroke-width="0.6"/>')
    svg.append(f'  <text x="43" y="18" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.2" font-weight="bold" text-anchor="middle" fill="#1A569A">xArm 1S Base</text>')
    svg.append(f'  <text x="43" y="22" font-family="DejaVu Sans, Arial, sans-serif" font-size="1.8" text-anchor="middle" fill="#444444">(175, 375) South ▼</text>')
    
    # Picking zone
    svg.append(f'  <rect x="18" y="27" width="50" height="20" fill="#FFFFFF" stroke="#999999" stroke-width="0.5" stroke-dasharray="1.5,1.5"/>')
    svg.append(f'  <text x="43" y="38" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.2" font-weight="bold" text-anchor="middle" fill="#666666">Picking Arena (300 × 400 mm)</text>')
    
    # ArUco Marker #0 (Top-Left)
    svg.append(f'  <rect x="6" y="20" width="7" height="7" fill="#000000" stroke="#000000"/>')
    svg.append(f'  <text x="14.5" y="24.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" fill="#000000">#0 (TL)</text>')
    
    # ArUco Marker #1 (Top-Right)
    svg.append(f'  <rect x="73" y="20" width="7" height="7" fill="#000000" stroke="#000000"/>')
    svg.append(f'  <text x="71.5" y="24.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" text-anchor="end" fill="#000000">#1 (TR)</text>')
    
    # ArUco Marker #2 (Bottom-Left)
    svg.append(f'  <rect x="6" y="38" width="7" height="7" fill="#000000" stroke="#000000"/>')
    svg.append(f'  <text x="14.5" y="42.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" fill="#000000">#2 (BL)</text>')
    
    # ArUco Marker #3 (Bottom-Right)
    svg.append(f'  <rect x="73" y="38" width="7" height="7" fill="#000000" stroke="#000000"/>')
    svg.append(f'  <text x="71.5" y="42.5" font-family="DejaVu Sans, Arial, sans-serif" font-size="2.0" font-weight="bold" text-anchor="end" fill="#000000">#3 (BR)</text>')
    
    svg.append('</svg>')
    return '\n'.join(svg)


def render_html_document() -> str:
    """Builds the complete HTML document containing Page 1 (Primary) and Page 2 (Spares)."""
    ruler_svg = build_ruler_svg(100.0)
    arena_map_svg = build_arena_map_svg()
    
    # Marker SVGs
    m0_svg = generate_marker_svg(0)
    m1_svg = generate_marker_svg(1)
    m2_svg = generate_marker_svg(2)
    m3_svg = generate_marker_svg(3)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>xArm 1S Arena - ArUco Calibration Markers (50x50 mm)</title>
<style>
@page {{
    size: A4 portrait;
    margin: 0;
}}
* {{
    box-sizing: border-box;
    -webkit-print-color-adjust: exact !important;
    print-color-adjust: exact !important;
}}
body {{
    margin: 0;
    padding: 0;
    font-family: 'DejaVu Sans', 'Arial', Helvetica, sans-serif;
    color: #111111;
    background-color: #FFFFFF;
}}
.page {{
    width: 210mm;
    height: 297mm;
    position: relative;
    padding: 9mm 12mm;
    page-break-after: always;
    overflow: hidden;
    background-color: #FFFFFF;
}}
.page:last-child {{
    page-break-after: avoid;
}}

/* Header */
.header {{
    text-align: center;
    border-bottom: 0.8mm solid #222222;
    padding-bottom: 2mm;
    margin-bottom: 3mm;
}}
.header h1 {{
    margin: 0 0 1.5mm 0;
    font-size: 4.8mm;
    font-weight: 800;
    letter-spacing: -0.2px;
    color: #0F172A;
}}
.badge-bar {{
    display: flex;
    justify-content: center;
    gap: 4mm;
    font-size: 2.3mm;
    font-weight: 600;
    color: #334155;
    margin-bottom: 2.5mm;
}}
.badge {{
    background-color: #F1F5F9;
    border: 0.3mm solid #CBD5E1;
    padding: 0.8mm 2.5mm;
    border-radius: 1mm;
}}
.badge-warning {{
    background-color: #FEF3C7;
    border-color: #F59E0B;
    color: #92400E;
}}

/* Ruler container */
.ruler-container {{
    display: flex;
    justify-content: center;
    margin-bottom: 3.5mm;
}}

/* Markers Grid */
.markers-grid {{
    display: grid;
    grid-template-columns: 1fr 1fr;
    column-gap: 14mm;
    row-gap: 5mm;
    margin-bottom: 4mm;
}}
.marker-card {{
    display: flex;
    flex-direction: column;
    align-items: center;
    page-break-inside: avoid;
}}
.marker-frame {{
    width: 60mm;
    height: 60mm;
    position: relative;
    display: flex;
    align-items: center;
    justify-content: center;
    background: #FFFFFF;
    margin-bottom: 2mm;
}}
.marker-svg {{
    display: block;
}}
.meta-box {{
    width: 76mm;
    background-color: #F8FAFC;
    border: 0.35mm solid #E2E8F0;
    border-left: 1.2mm solid #0284C7;
    border-radius: 1mm;
    padding: 1.8mm 2.5mm;
    font-size: 2.2mm;
    line-height: 1.45;
}}
.meta-title {{
    font-size: 2.8mm;
    font-weight: 800;
    color: #0F172A;
    margin-bottom: 0.6mm;
    display: flex;
    justify-content: space-between;
}}
.meta-row {{
    display: flex;
    justify-content: space-between;
    color: #475569;
}}
.meta-val {{
    font-weight: 700;
    color: #0F172A;
}}
.meta-dest {{
    color: #0369A1;
    font-weight: 700;
}}

/* Bottom Panel */
.bottom-panel {{
    display: grid;
    grid-template-columns: 1.05fr 0.95fr;
    column-gap: 5mm;
    border-top: 0.6mm solid #CBD5E1;
    padding-top: 3mm;
}}
.guide-box {{
    background-color: #FAFAFA;
    border: 0.3mm solid #E2E8F0;
    border-radius: 1.5mm;
    padding: 2.5mm 3.5mm;
}}
.guide-title {{
    font-size: 2.7mm;
    font-weight: 800;
    color: #0F172A;
    margin-bottom: 1.5mm;
    display: flex;
    align-items: center;
    gap: 1.5mm;
}}
.guide-list {{
    margin: 0;
    padding-left: 4mm;
    font-size: 2.1mm;
    line-height: 1.5;
    color: #334155;
}}
.guide-list li {{
    margin-bottom: 1mm;
}}
.guide-list strong {{
    color: #0F172A;
}}
.map-box {{
    display: flex;
    justify-content: center;
    align-items: center;
}}

/* Cutout notice */
.cutout-guarantee {{
    margin-top: 1.5mm;
    font-size: 2.0mm;
    color: #059669;
    font-weight: 700;
    display: flex;
    align-items: center;
    gap: 1mm;
}}

/* Page footer */
.page-footer {{
    position: absolute;
    bottom: 3.5mm;
    left: 12mm;
    right: 12mm;
    display: flex;
    justify-content: space-between;
    font-size: 2.0mm;
    color: #94A3B8;
    border-top: 0.25mm solid #E2E8F0;
    padding-top: 1.2mm;
}}
</style>
</head>
<body>

<!-- PAGE 1: PRIMARY ARENA CALIBRATION SET -->
<div class="page">
    <div class="header">
        <h1>xArm 1S Arena ArUco Calibration Markers (50 × 50 mm)</h1>
        <div class="badge-bar">
            <span class="badge badge-warning">⚠ PRINT SETTING: 100% Scale / Actual Size (NO "Fit to Page")</span>
            <span class="badge">Standard: ISO A4 (210 × 297 mm)</span>
            <span class="badge">Dictionary: DICT_4X4_50</span>
            <span class="badge">Size: Exactly 5.0 × 5.0 cm</span>
        </div>
        <div class="ruler-container">
            {ruler_svg}
        </div>
    </div>

    <div class="markers-grid">
        <!-- Marker 0 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m0_svg}
            </div>
            <div class="meta-box" style="border-left-color: #DC2626;">
                <div class="meta-title">
                    <span>ArUco #0 — Top-Left</span>
                    <span class="meta-dest">Pixel: (0, 0)</span>
                </div>
                <div class="meta-row"><span>Cell Location:</span><span class="meta-val">Row 3, Col 0</span></div>
                <div class="meta-row"><span>Coordinates:</span><span class="meta-val">X = 25.0 mm, Y = 425.0 mm</span></div>
                <div class="meta-row"><span>Cut Size:</span><span class="meta-val">50.0 mm (flush) or 60.0 mm (quiet)</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Marker 1 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m1_svg}
            </div>
            <div class="meta-box" style="border-left-color: #16A34A;">
                <div class="meta-title">
                    <span>ArUco #1 — Top-Right</span>
                    <span class="meta-dest">Pixel: (600, 0)</span>
                </div>
                <div class="meta-row"><span>Cell Location:</span><span class="meta-val">Row 3, Col 6</span></div>
                <div class="meta-row"><span>Coordinates:</span><span class="meta-val">X = 325.0 mm, Y = 425.0 mm</span></div>
                <div class="meta-row"><span>Cut Size:</span><span class="meta-val">50.0 mm (flush) or 60.0 mm (quiet)</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Marker 2 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m2_svg}
            </div>
            <div class="meta-box" style="border-left-color: #2563EB;">
                <div class="meta-title">
                    <span>ArUco #2 — Bottom-Left</span>
                    <span class="meta-dest">Pixel: (0, 800)</span>
                </div>
                <div class="meta-row"><span>Cell Location:</span><span class="meta-val">Row 11, Col 0</span></div>
                <div class="meta-row"><span>Coordinates:</span><span class="meta-val">X = 25.0 mm, Y = 25.0 mm</span></div>
                <div class="meta-row"><span>Cut Size:</span><span class="meta-val">50.0 mm (flush) or 60.0 mm (quiet)</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Marker 3 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m3_svg}
            </div>
            <div class="meta-box" style="border-left-color: #9333EA;">
                <div class="meta-title">
                    <span>ArUco #3 — Bottom-Right</span>
                    <span class="meta-dest">Pixel: (600, 800)</span>
                </div>
                <div class="meta-row"><span>Cell Location:</span><span class="meta-val">Row 11, Col 6</span></div>
                <div class="meta-row"><span>Coordinates:</span><span class="meta-val">X = 325.0 mm, Y = 25.0 mm</span></div>
                <div class="meta-row"><span>Cut Size:</span><span class="meta-val">50.0 mm (flush) or 60.0 mm (quiet)</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>
    </div>

    <!-- Bottom Instructions & Arena Layout -->
    <div class="bottom-panel">
        <div class="guide-box">
            <div class="guide-title">✂ Cutting & Mounting Instructions (Coplanar Z = 0.0 mm)</div>
            <ol class="guide-list">
                <li><strong>Verify Scale First:</strong> Measure the 100 mm ruler above with a physical ruler. [0 to 50 mm] must be exactly 5.0 cm.</li>
                <li><strong>Cutting Options:</strong>
                    <br>• <em>Option A (Recommended):</em> Cut along the outer dashed line (60×60 mm) to retain the 5 mm white quiet zone.
                    <br>• <em>Option B:</em> Cut directly along the black marker edge (50×50 mm) if pasting onto white MDF board.
                </li>
                <li><strong>Clean Cutouts:</strong> Text and labels are strictly outside the cut lines. The marker cutout contains zero text.</li>
                <li><strong>Coplanar Mounting:</strong> Paste flat onto the arena baseplate at Z = 0.0 mm using double-sided adhesive or spray glue. Ensure no bubbles or curl.</li>
            </ol>
        </div>

        <div class="map-box">
            {arena_map_svg}
        </div>
    </div>

    <div class="page-footer">
        <span>Sem-5 Mini Project: Robotic Sorting Arm — Overhead Vision Homography System</span>
        <span>Page 1 of 2 (Primary Arena Set)</span>
    </div>
</div>


<!-- PAGE 2: SPARE / BACKUP CALIBRATION SET -->
<div class="page">
    <div class="header">
        <h1>xArm 1S Arena ArUco Markers — Spare / Backup Set</h1>
        <div class="badge-bar">
            <span class="badge badge-warning">⚠ PRINT SETTING: 100% Scale / Actual Size (NO "Fit to Page")</span>
            <span class="badge">Standard: ISO A4 (210 × 297 mm)</span>
            <span class="badge">Dictionary: DICT_4X4_50</span>
            <span class="badge">4× Backup Markers</span>
        </div>
        <div class="ruler-container">
            {ruler_svg}
        </div>
    </div>

    <div class="markers-grid">
        <!-- Spare Marker 0 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m0_svg}
            </div>
            <div class="meta-box" style="border-left-color: #DC2626;">
                <div class="meta-title">
                    <span>SPARE ArUco #0</span>
                    <span class="meta-dest">ID: 0</span>
                </div>
                <div class="meta-row"><span>Target Role:</span><span class="meta-val">Top-Left (Row 3, Col 0)</span></div>
                <div class="meta-row"><span>World Position:</span><span class="meta-val">X = 25.0 mm, Y = 425.0 mm</span></div>
                <div class="meta-row"><span>Marker Dimensions:</span><span class="meta-val">50.0 × 50.0 mm</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Spare Marker 1 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m1_svg}
            </div>
            <div class="meta-box" style="border-left-color: #16A34A;">
                <div class="meta-title">
                    <span>SPARE ArUco #1</span>
                    <span class="meta-dest">ID: 1</span>
                </div>
                <div class="meta-row"><span>Target Role:</span><span class="meta-val">Top-Right (Row 3, Col 6)</span></div>
                <div class="meta-row"><span>World Position:</span><span class="meta-val">X = 325.0 mm, Y = 425.0 mm</span></div>
                <div class="meta-row"><span>Marker Dimensions:</span><span class="meta-val">50.0 × 50.0 mm</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Spare Marker 2 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m2_svg}
            </div>
            <div class="meta-box" style="border-left-color: #2563EB;">
                <div class="meta-title">
                    <span>SPARE ArUco #2</span>
                    <span class="meta-dest">ID: 2</span>
                </div>
                <div class="meta-row"><span>Target Role:</span><span class="meta-val">Bottom-Left (Row 11, Col 0)</span></div>
                <div class="meta-row"><span>World Position:</span><span class="meta-val">X = 25.0 mm, Y = 25.0 mm</span></div>
                <div class="meta-row"><span>Marker Dimensions:</span><span class="meta-val">50.0 × 50.0 mm</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>

        <!-- Spare Marker 3 -->
        <div class="marker-card">
            <div class="marker-frame">
                {m3_svg}
            </div>
            <div class="meta-box" style="border-left-color: #9333EA;">
                <div class="meta-title">
                    <span>SPARE ArUco #3</span>
                    <span class="meta-dest">ID: 3</span>
                </div>
                <div class="meta-row"><span>Target Role:</span><span class="meta-val">Bottom-Right (Row 11, Col 6)</span></div>
                <div class="meta-row"><span>World Position:</span><span class="meta-val">X = 325.0 mm, Y = 25.0 mm</span></div>
                <div class="meta-row"><span>Marker Dimensions:</span><span class="meta-val">50.0 × 50.0 mm</span></div>
                <div class="cutout-guarantee">✓ ZERO text on cutout marker</div>
            </div>
        </div>
    </div>

    <!-- Bottom Panel for Page 2 -->
    <div class="bottom-panel">
        <div class="guide-box" style="grid-column: span 2;">
            <div class="guide-title">📌 Backup Notes & Camera Homography Tips</div>
            <ol class="guide-list">
                <li><strong>Why Backup Markers?</strong> In physical robotics setups, tape peel, adhesive slippage, or surface scuffs can degrade sub-pixel corner refinement. Keep these 4 spare markers handy.</li>
                <li><strong>Sub-Pixel Refinement:</strong> The ROS 2 vision pipeline uses <code>CORNER_REFINE_SUBPIX</code> with <code>cv2.aruco.DICT_4X4_50</code>. Clean, sharp cuts with flat adhesion ensure sub-2 mm centroid error.</li>
                <li><strong>Lighting Consideration:</strong> Ensure uniform, non-glare overhead illumination. If using glossy sticker paper, avoid direct specular reflections from spotlights into the overhead camera lens.</li>
            </ol>
        </div>
    </div>

    <div class="page-footer">
        <span>Sem-5 Mini Project: Robotic Sorting Arm — Overhead Vision Homography System</span>
        <span>Page 2 of 2 (Spare / Backup Set)</span>
    </div>
</div>

</body>
</html>
"""
    return html


def generate_single_page_html() -> str:
    """Builds a standalone 1-page HTML document containing only Page 1 for rapid 1-sheet printing."""
    doc = render_html_document()
    # Strip everything after page 1
    idx = doc.find("<!-- PAGE 2:")
    if idx != -1:
        end_idx = doc.rfind("</body>")
        page1_doc = doc[:idx] + "</body>\n</html>"
        # Update page footer
        page1_doc = page1_doc.replace("Page 1 of 2 (Primary Arena Set)", "Arena Calibration Set (Ready to Cut & Mount)")
        return page1_doc
    return doc


def generate_individual_marker_pngs():
    """Generates individual 600x600 px high-res PNGs of each marker with quiet zone."""
    marker_px = 600
    border_px = 60
    
    for marker_id in range(4):
        img = cv2.aruco.generateImageMarker(ARUCO_DICT, marker_id, marker_px, borderBits=1)
        padded = np.full((marker_px + 2 * border_px, marker_px + 2 * border_px), 255, dtype=np.uint8)
        padded[border_px:border_px + marker_px, border_px:border_px + marker_px] = img
        
        # Save clean marker without text
        clean_path = os.path.join(OUTPUT_DIR, f"marker_{marker_id}_50x50mm_clean.png")
        cv2.imwrite(clean_path, padded)
        
        # Also save marked version with label outside the quiet zone
        h, w = padded.shape
        annotated = np.full((h + 80, w), 255, dtype=np.uint8)
        annotated[:h, :w] = padded
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)
        
        # Text label strictly below
        meta = MARKER_METADATA[marker_id]
        cv2.putText(annotated_bgr, f"ArUco ID #{marker_id} ({meta['role']}) - DICT_4X4_50", 
                    (20, h + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(annotated_bgr, f"Position: {meta['cell']} | Size: 50.0 x 50.0 mm | Z = 0.0 mm", 
                    (20, h + 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1, cv2.LINE_AA)
        
        ann_path = os.path.join(OUTPUT_DIR, f"marker_{marker_id}_50x50mm_annotated.png")
        cv2.imwrite(ann_path, annotated_bgr)
        print(f"Generated individual marker: {clean_path}")


def main():
    print(f"Generating printable ArUco sheets in '{OUTPUT_DIR}'...")
    
    # 1. HTML Documents
    html_2page = render_html_document()
    html_1page = generate_single_page_html()
    
    html_path_2p = os.path.join(OUTPUT_DIR, "aruco_markers_A4_sheet.html")
    with open(html_path_2p, "w", encoding="utf-8") as f:
        f.write(html_2page)
    print(f"✓ Saved HTML sheet: {html_path_2p}")
    
    # 2. Vector PDFs via WeasyPrint
    pdf_path_2p = os.path.join(OUTPUT_DIR, "aruco_markers_A4_50x50mm.pdf")
    weasyprint.HTML(string=html_2page).write_pdf(pdf_path_2p)
    print(f"✓ Generated 2-Page Vector PDF: {pdf_path_2p}")
    
    pdf_path_1p = os.path.join(OUTPUT_DIR, "aruco_markers_A4_single_page_50x50mm.pdf")
    weasyprint.HTML(string=html_1page).write_pdf(pdf_path_1p)
    print(f"✓ Generated 1-Page Vector PDF: {pdf_path_1p}")
    
    # 3. Render 300 DPI high-resolution PNGs from the PDF using pdftoppm
    prefix = os.path.join(OUTPUT_DIR, "aruco_sheet_300dpi")
    os.system(f"pdftoppm -png -r 300 '{pdf_path_2p}' '{prefix}'")
    
    # Rename generated files to standard names
    p1 = f"{prefix}-1.png"
    p2 = f"{prefix}-2.png"
    target_p1 = os.path.join(OUTPUT_DIR, "aruco_markers_A4_page1_300dpi.png")
    target_p2 = os.path.join(OUTPUT_DIR, "aruco_markers_A4_page2_300dpi.png")
    
    if os.path.exists(p1):
        os.rename(p1, target_p1)
        print(f"✓ Rendered 300 DPI PNG Page 1: {target_p1}")
    if os.path.exists(p2):
        os.rename(p2, target_p2)
        print(f"✓ Rendered 300 DPI PNG Page 2: {target_p2}")
        
    # 4. Generate individual clean marker PNGs
    generate_individual_marker_pngs()
    
    # 5. Verification Test: Run OpenCV ArUco detector on Page 1 PNG
    if os.path.exists(target_p1):
        test_img = cv2.imread(target_p1)
        detector = cv2.aruco.ArucoDetector(ARUCO_DICT)
        corners, ids, rejected = detector.detectMarkers(test_img)
        detected_list = sorted(ids.flatten().tolist()) if ids is not None else []
        print(f"✓ Verification: OpenCV detected ArUco IDs on printable sheet: {detected_list}")
        assert set(detected_list) == {0, 1, 2, 3}, f"Failed detection: {detected_list}"
        print("✓ SUCCESS: All 4 markers (IDs 0, 1, 2, 3) detected with 100% precision!")

if __name__ == "__main__":
    main()
