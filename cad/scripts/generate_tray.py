#!/usr/bin/env python3
"""
Parametric 3D CAD Model Generator for 2x2 Color Sorting Tray.
Generates STEP and STL files for 3D printing and CAD integration.
"""
import os
from build123d import *

# Dimensions in mm
OUTER_WIDTH = 80.0
OUTER_DEPTH = 80.0
TOTAL_HEIGHT = 35.0
FLOOR_THICKNESS = 3.0
WALL_THICKNESS = 2.0
DIVIDER_THICKNESS = 2.0
CORNER_RADIUS_OUTER = 3.0
CORNER_RADIUS_INNER = 2.5
TOP_CHAMFER = 0.8

# Calculate pocket sizes
# (80 - 2*2 - 2) / 2 = 37.0 mm
pocket_w = (OUTER_WIDTH - 2 * WALL_THICKNESS - DIVIDER_THICKNESS) / 2.0
pocket_d = (OUTER_DEPTH - 2 * WALL_THICKNESS - DIVIDER_THICKNESS) / 2.0
pocket_h = TOTAL_HEIGHT - FLOOR_THICKNESS

print(f"Creating 2x2 Sorting Tray: {OUTER_WIDTH}x{OUTER_DEPTH}x{TOTAL_HEIGHT} mm")
print(f"Pocket dimensions: {pocket_w:.1f}x{pocket_d:.1f}x{pocket_h:.1f} mm")

with BuildPart() as tray:
    # 1. Main outer body with rounded vertical corners
    with BuildSketch() as outer_sk:
        RectangleRounded(OUTER_WIDTH, OUTER_DEPTH, radius=CORNER_RADIUS_OUTER)
    extrude(amount=TOTAL_HEIGHT)
    
    # 2. Cut 4 quadrant pockets
    # Pocket centers:
    # X offsets: +/- (DIVIDER_THICKNESS/2 + pocket_w/2)
    # Y offsets: +/- (DIVIDER_THICKNESS/2 + pocket_d/2)
    cx = DIVIDER_THICKNESS / 2.0 + pocket_w / 2.0
    cy = DIVIDER_THICKNESS / 2.0 + pocket_d / 2.0
    
    pocket_positions = [
        (-cx, -cy),
        ( cx, -cy),
        (-cx,  cy),
        ( cx,  cy),
    ]
    
    for px, py in pocket_positions:
        with Locations((px, py, FLOOR_THICKNESS)):
            with BuildSketch() as pocket_sk:
                RectangleRounded(pocket_w, pocket_d, radius=CORNER_RADIUS_INNER)
            extrude(amount=pocket_h + 1.0, mode=Mode.SUBTRACT)

    # 3. Add top rim chamfer to guide falling blocks smoothly into slots
    top_edges = tray.edges().filter_by(Axis.Z, reverse=True).filter_by(GeomType.LINE)
    # Filter edges at the top surface Z = TOTAL_HEIGHT
    rim_edges = [e for e in tray.edges() if abs(e.center().Z - TOTAL_HEIGHT) < 0.1]
    if rim_edges:
        try:
            chamfer(rim_edges, length=TOP_CHAMFER)
        except Exception as e:
            print(f"Top chamfer skipped: {e}")

part = tray.part
print(f"Volume: {part.volume:.1f} mm^3")
print(f"Bounding box: {part.bounding_box()}")

MODELS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models"))
os.makedirs(MODELS_DIR, exist_ok=True)
step_path = os.path.join(MODELS_DIR, "sorting_tray_2x2.step")
stl_path = os.path.join(MODELS_DIR, "sorting_tray_2x2.stl")

export_step(part, step_path)
export_stl(part, stl_path)
print(f"Exported {step_path} and {stl_path} successfully!")
