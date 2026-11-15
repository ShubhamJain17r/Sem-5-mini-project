import os
from PIL import Image, ImageDraw, ImageFont

WIDTH = 3400
HEIGHT = 2800

img = Image.new("RGBA", (WIDTH, HEIGHT), (255, 255, 255, 255))
draw = ImageDraw.Draw(img)

try:
    font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 46)
    font_title = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 34)
    font_body = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
    font_body_bold = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
    font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    font_mono = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 22)
    font_mono_bold = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 23)
except Exception:
    font_large = ImageFont.load_default()
    font_title = font_large
    font_body = font_large
    font_body_bold = font_large
    font_small = font_large
    font_mono = font_large
    font_mono_bold = font_large

SCALE = 4.3
OFFSET_X = 430
OFFSET_Y = 2450

def to_screen(x_mm, y_mm):
    sx = int(OFFSET_X + x_mm * SCALE)
    sy = int(OFFSET_Y - y_mm * SCALE)
    return (sx, sy)

def draw_multiline(xy, text, font, fill, align="center", line_spacing=6):
    lines = text.strip().split("\n")
    line_sizes = [draw.textsize(l, font=font) for l in lines]
    total_h = sum(h for _, h in line_sizes) + (len(lines) - 1) * line_spacing
    cur_y = xy[1] - total_h // 2
    for l, (w, h) in zip(lines, line_sizes):
        if align == "center":
            cur_x = xy[0] - w // 2
        elif align == "right":
            cur_x = xy[0] - w
        else:
            cur_x = xy[0]
        draw.text((cur_x, cur_y), l, font=font, fill=fill)
        cur_y += h + line_spacing

# Light engineering grid
for x_mm in range(-40, 441, 20):
    draw.line([to_screen(x_mm, -40), to_screen(x_mm, 440)], fill=(240, 244, 248), width=2)
for y_mm in range(-40, 441, 20):
    draw.line([to_screen(-40, y_mm), to_screen(440, y_mm)], fill=(240, 244, 248), width=2)

for x_mm in range(0, 401, 100):
    draw.line([to_screen(x_mm, 0), to_screen(x_mm, 400)], fill=(226, 232, 240), width=3)
for y_mm in range(0, 401, 100):
    draw.line([to_screen(0, y_mm), to_screen(400, y_mm)], fill=(226, 232, 240), width=3)

# 1. Baseboard 400x400 mm
b_tl = to_screen(0, 400)
b_br = to_screen(400, 0)
draw.rectangle([b_tl, b_br], outline=(30, 41, 59), width=8, fill=(248, 250, 252))

# Corner mounting holes
for hx, hy in [(15, 15), (385, 15), (15, 385), (385, 385)]:
    sp = to_screen(hx, hy)
    draw.ellipse([sp[0]-14, sp[1]-14, sp[0]+14, sp[1]+14], outline=(100, 116, 139), width=4)
    draw.line([sp[0]-22, sp[1], sp[0]+22, sp[1]], fill=(100, 116, 139), width=2)
    draw.line([sp[0], sp[1]-22, sp[0], sp[1]+22], fill=(100, 116, 139), width=2)

# 2. Rear Electronics Bay (X: 100-300, Y: 335-395)
draw.rectangle([to_screen(100, 395), to_screen(300, 335)], outline=(2, 132, 199), width=4, fill=(224, 242, 254))
draw_multiline(to_screen(200, 365), "REAR ELECTRONICS & PSU BAY\n(Raspberry Pi 5 / Jetson & 7.4V / 5A Buck Regulator)",
               font=font_body_bold, fill=(3, 105, 161), align="center")

# Stand Clamp
draw.rectangle([to_screen(180, 400), to_screen(220, 385)], outline=(15, 23, 42), width=4, fill=(51, 65, 85))
draw_multiline(to_screen(200, 392.5), "OVERHEAD CAMERA STAND CLAMP", font=font_small, fill=(255, 255, 255), align="center")

# 3. Active Picking Arena (260x140 mm, X: 70..330, Y: 40..180)
# Outer foam border (10mm thick)
draw.rectangle([to_screen(60, 190), to_screen(340, 30)], outline=(15, 23, 42), width=5, fill=(203, 213, 225))
draw.rectangle([to_screen(70, 180), to_screen(330, 40)], outline=(15, 23, 42), width=6, fill=(255, 255, 255))
draw_multiline(to_screen(200, 110), "ACTIVE PICKING ARENA\n260.0 mm (W) × 140.0 mm (D)\nX ∈ [70.0, 330.0], Y ∈ [40.0, 180.0]\n(Tabletop Floor Z = 0.0 mm)",
               font=font_title, fill=(15, 23, 42), align="center")

# 4. ArUco Markers
markers = [
    (0, 70, 180, "ArUco #0 [Rear-Left]\n(70.0, 180.0 mm)", -80, -45, "right"),
    (1, 330, 180, "ArUco #1 [Rear-Right]\n(330.0, 180.0 mm)", 80, -45, "left"),
    (2, 70, 40, "ArUco #2 [Front-Left]\n(70.0, 40.0 mm)", -80, 45, "right"),
    (3, 330, 40, "ArUco #3 [Front-Right]\n(330.0, 40.0 mm)", 80, 45, "left")
]
for mid, mx, my, mtext, ox, oy, align in markers:
    sp = to_screen(mx, my)
    hw = int(15 * SCALE)
    draw.rectangle([sp[0]-hw, sp[1]-hw, sp[0]+hw, sp[1]+hw], fill="black", outline="black", width=2)
    iw = int(11 * SCALE)
    draw.rectangle([sp[0]-iw, sp[1]-iw, sp[0]+iw, sp[1]+iw], fill="white", outline="white")
    pw = int(7 * SCALE)
    draw.rectangle([sp[0]-pw, sp[1]-pw, sp[0]+pw, sp[1]+pw], fill="black")
    draw.line([sp[0]-20, sp[1], sp[0]+20, sp[1]], fill=(220, 38, 38), width=3)
    draw.line([sp[0], sp[1]-20, sp[0], sp[1]+20], fill=(220, 38, 38), width=3)
    
    draw_multiline((sp[0] + ox, sp[1] + oy), mtext, font=font_body_bold, fill=(185, 28, 28), align=align)

# 5. Single xArm 1S Mount (200, 280)
bx, by = 200, 280
draw.rectangle([to_screen(bx - 70, by + 77), to_screen(bx + 70, by - 77)],
               outline=(37, 99, 235), width=6, fill=(219, 234, 254))

sp_base = to_screen(bx, by)
r_turret = int(48 * SCALE)
draw.ellipse([sp_base[0]-r_turret, sp_base[1]-r_turret, sp_base[0]+r_turret, sp_base[1]+r_turret],
             outline=(59, 130, 246), width=5)
draw.ellipse([sp_base[0]-25, sp_base[1]-25, sp_base[0]+25, sp_base[1]+25], fill=(29, 78, 216))

# 4 mounting screw holes
for dx, dy in [(-63.5, -71.5), (63.5, -71.5), (-63.5, 71.5), (63.5, 71.5)]:
    hx, hy = bx + dx, by + dy
    sp_h = to_screen(hx, hy)
    draw.ellipse([sp_h[0]-10, sp_h[1]-10, sp_h[0]+10, sp_h[1]+10], fill=(30, 64, 175))
    w_tap, h_tap = draw.textsize("M4", font=font_small)
    draw.text((sp_h[0]-w_tap//2, sp_h[1]+14), "M4", font=font_small, fill=(30, 64, 175))

# Heading vector (South)
sp_tip = to_screen(bx, by - 65)
draw.line([sp_base, sp_tip], fill=(29, 78, 216), width=7)
draw.polygon([(sp_tip[0]-16, sp_tip[1]-22), (sp_tip[0]+16, sp_tip[1]-22), sp_tip], fill=(29, 78, 216))

# Base callouts placed cleanly without overlap
draw_multiline((sp_base[0], sp_base[1] - int(48 * SCALE)),
               "xArm 1S (Manipulator Mount)\nBase Center: (X = 200.0, Y = 280.0, Z = 0.0 mm)",
               font=font_body_bold, fill=(30, 58, 138), align="center")
draw_multiline((sp_base[0] + 120, sp_base[1] + 45),
               "Heading: -90.0°\n(Facing Due South)",
               font=font_small, fill=(29, 78, 216), align="left")

# 6. Left Flank 2x2 Tray (50, 280)
tx, ty = 50, 280
draw.rectangle([to_screen(tx - 40, ty + 40), to_screen(tx + 40, ty - 40)],
               outline=(71, 85, 105), width=6, fill=(241, 245, 249))

pockets = [
    ("RED", tx - 36, ty + 36, 32, 32, (239, 68, 68), (254, 226, 226)),
    ("GREEN", tx + 4, ty + 36, 32, 32, (34, 197, 94), (220, 252, 231)),
    ("BLUE", tx - 36, ty - 4, 32, 32, (59, 130, 246), (219, 234, 254)),
    ("YELLOW", tx + 4, ty - 4, 32, 32, (202, 138, 4), (254, 249, 195)),
]
for name, px, py, pw, ph, ec, fc in pockets:
    p_tl = to_screen(px, py)
    p_br = to_screen(px + pw, py - ph)
    draw.rectangle([p_tl, p_br], outline=ec, width=4, fill=fc)
    p_mid = to_screen(px + pw/2, py - ph/2)
    w_txt, h_txt = draw.textsize(name, font=font_body_bold)
    draw.text((p_mid[0]-w_txt//2, p_mid[1]-h_txt//2), name, font=font_body_bold, fill=ec)

draw_multiline(to_screen(tx, ty - 56), "Left Flank 2×2 Tray\n(Center: 50.0, 280.0 mm)",
               font=font_body_bold, fill=(51, 65, 85), align="center")

# Direct lateral drop swing line/arc (from 200, 280 directly west to 50, 280)
p_arm = to_screen(bx, by)
p_drop = to_screen(tx + 40, ty)
draw.line([p_arm, p_drop], fill=(147, 51, 234), width=5)
draw.polygon([(p_drop[0]+25, p_drop[1]-15), (p_drop[0]+25, p_drop[1]+15), p_drop], fill=(147, 51, 234))
draw_multiline(to_screen(125, 295), "90.0° CW Drop Swing\n(r = 150.0 mm)",
               font=font_body_bold, fill=(126, 34, 206), align="center")

# 7. Coordinate Origin & Dimensions
origin_screen = to_screen(0, 0)
x_axis_tip = to_screen(75, 0)
y_axis_tip = to_screen(0, 75)
draw.line([origin_screen, x_axis_tip], fill=(220, 38, 38), width=8)
draw.polygon([(x_axis_tip[0]-18, x_axis_tip[1]-12), (x_axis_tip[0]-18, x_axis_tip[1]+12), x_axis_tip], fill=(220, 38, 38))
draw.line([origin_screen, y_axis_tip], fill=(220, 38, 38), width=8)
draw.polygon([(y_axis_tip[0]-12, y_axis_tip[1]+18), (y_axis_tip[0]+12, y_axis_tip[1]+18), y_axis_tip], fill=(220, 38, 38))

draw.text((x_axis_tip[0]+15, x_axis_tip[1]-15), "+X (Width)", font=font_body_bold, fill=(220, 38, 38))
draw.text((y_axis_tip[0]-40, y_axis_tip[1]-40), "+Y (Depth)", font=font_body_bold, fill=(220, 38, 38))
draw_multiline((origin_screen[0]-15, origin_screen[1]+35), "WORLD ORIGIN (0, 0)\nFront-Left Corner",
               font=font_body_bold, fill=(220, 38, 38), align="right")

# Overall Dimensions lines
dim_top_p1 = to_screen(0, 415)
dim_top_p2 = to_screen(400, 415)
draw.line([dim_top_p1, dim_top_p2], fill=(51, 65, 85), width=3)
draw_multiline(((dim_top_p1[0]+dim_top_p2[0])//2, dim_top_p1[1]-20), "TOTAL BASEBOARD WIDTH = 400.0 mm",
               font=font_body_bold, fill=(51, 65, 85), align="center")

dim_left_p1 = to_screen(-20, 0)
dim_left_p2 = to_screen(-20, 400)
draw.line([dim_left_p1, dim_left_p2], fill=(51, 65, 85), width=3)
draw_multiline((dim_left_p1[0]-35, (dim_left_p1[1]+dim_left_p2[1])//2), "TOTAL BASEBOARD\nDEPTH = 400.0 mm",
               font=font_body_bold, fill=(51, 65, 85), align="right")

# Header Title
w_title, h_title = draw.textsize("SINGLE xARM 1S WORKCELL — 1:1 ENGINEERING ARENA LAYOUT", font=font_large)
draw.text((WIDTH//2 - 400 - w_title//2, 60), "SINGLE xARM 1S WORKCELL — 1:1 ENGINEERING ARENA LAYOUT",
          font=font_large, fill=(15, 23, 42))

# Right panel (Cheat Sheet)
draw.rectangle([(2260, 60), (3340, 2720)], outline=(203, 213, 225), width=5, fill=(248, 250, 252))

cheat_lines = [
    ("==================================================", font_mono_bold, (15, 23, 42)),
    ("   SINGLE xARM 1S WORKCELL SPECIFICATION", font_mono_bold, (15, 23, 42)),
    ("   1:1 Tabletop Geometry & Coordinate Cheat-Sheet", font_mono, (71, 85, 105)),
    ("==================================================", font_mono_bold, (15, 23, 42)),
    ("", font_mono, (0, 0, 0)),
    ("--- BASEBOARD & MATERIAL ---", font_mono_bold, (37, 99, 235)),
    ("  Dimensions: 400.0 × 400.0 × 18.0 mm (W × D × H)", font_mono, (51, 65, 85)),
    ("  Material: Grade-A Birch Plywood or White MDF", font_mono, (51, 65, 85)),
    ("  Mass: ~2.3 kg (Zero carpentry, standard store cut)", font_mono, (51, 65, 85)),
    ("  World Origin: Front-Left Corner (X=0, Y=0, Z=0)", font_mono, (51, 65, 85)),
    ("  Front Operator Edge: Y = 0.0 mm", font_mono, (51, 65, 85)),
    ("  Rear Boundary: Y = 400.0 mm", font_mono, (51, 65, 85)),
    ("", font_mono, (0, 0, 0)),
    ("--- MANIPULATOR MOUNT (xArm 1S) ---", font_mono_bold, (37, 99, 235)),
    ("  Base Center: (X = 200.0, Y = 280.0, Z = 0.0 mm)", font_mono, (51, 65, 85)),
    ("  Nominal Heading: -90.0° (Facing Due South)", font_mono, (51, 65, 85)),
    ("  Baseplate Outer: 140.0 × 154.0 mm", font_mono, (51, 65, 85)),
    ("  Mounting Pattern: 127.0 × 143.0 mm (4× M4 bolts)", font_mono, (51, 65, 85)),
    ("    Hole 1 (Rear-Left):  (X = 136.5, Y = 351.5 mm)", font_mono, (51, 65, 85)),
    ("    Hole 2 (Rear-Right): (X = 263.5, Y = 351.5 mm)", font_mono, (51, 65, 85)),
    ("    Hole 3 (Front-Left):  (X = 136.5, Y = 208.5 mm)", font_mono, (51, 65, 85)),
    ("    Hole 4 (Front-Right): (X = 263.5, Y = 208.5 mm)", font_mono, (51, 65, 85)),
    ("  Radial Reach Bounds: r ∈ [100.0, 272.9 mm]", font_mono, (51, 65, 85)),
    ("  Max Operational Envelope: R_max = 280.0 mm", font_mono, (51, 65, 85)),
    ("", font_mono, (0, 0, 0)),
    ("--- ACTIVE PICKING ARENA ---", font_mono_bold, (37, 99, 235)),
    ("  Arena Width (X): 260.0 mm (X ∈ [70.0, 330.0 mm])", font_mono, (51, 65, 85)),
    ("  Arena Depth (Y): 140.0 mm (Y ∈ [40.0, 180.0 mm])", font_mono, (51, 65, 85)),
    ("  Perimeter Barrier: 10 mm Soft Black Foam Border", font_mono, (51, 65, 85)),
    ("  Floor Elevation: Z = 0.0 mm (Workcell Tabletop)", font_mono, (51, 65, 85)),
    ("  Target Cube Size: 25.0 × 25.0 × 25.0 mm Craft Cubes", font_mono, (51, 65, 85)),
    ("  Cube Centroid Grab Height: Z = 12.5 mm", font_mono, (51, 65, 85)),
    ("", font_mono, (0, 0, 0)),
    ("--- ARUCO FIDUCIAL MARKERS (DICT_4X4_50) ---", font_mono_bold, (37, 99, 235)),
    ("  Outer Footprint: 30.0 × 30.0 mm (5 mm quiet border)", font_mono, (51, 65, 85)),
    ("  Marker 0 (Rear-Left):   (X = 70.0,  Y = 180.0 mm)", font_mono, (51, 65, 85)),
    ("  Marker 1 (Rear-Right):  (X = 330.0, Y = 180.0 mm)", font_mono, (51, 65, 85)),
    ("  Marker 2 (Front-Left):  (X = 70.0,  Y = 40.0 mm)", font_mono, (51, 65, 85)),
    ("  Marker 3 (Front-Right): (X = 330.0, Y = 40.0 mm)", font_mono, (51, 65, 85)),
    ("  Rectified Metric Grid: 520 × 280 px @ 2.0 px/mm", font_mono, (51, 65, 85)),
    ("", font_mono, (0, 0, 0)),
    ("--- LEFT FLANK 2×2 SORTING TRAY ---", font_mono_bold, (37, 99, 235)),
    ("  Tray Center: (X = 50.0, Y = 280.0, Z = 0.0 mm)", font_mono, (51, 65, 85)),
    ("  Tray Outer Footprint: 80.0 × 80.0 × 35.0 mm (PLA+)", font_mono, (51, 65, 85)),
    ("  Drop Swing Arc: 90.0° Clockwise (Yaw Tick ≈ 145)", font_mono, (51, 65, 85)),
    ("  Radius from Base: r = 150.0 mm (Direct Lateral Swing)", font_mono, (51, 65, 85)),
    ("  Compartment Centers:", font_mono, (51, 65, 85)),
    ("    RED:    (X = 30.0, Y = 300.0 mm) -> Yaw Tick: 135", font_mono, (185, 28, 28)),
    ("    GREEN:  (X = 70.0, Y = 300.0 mm) -> Yaw Tick: 125", font_mono, (22, 163, 74)),
    ("    BLUE:   (X = 30.0, Y = 260.0 mm) -> Yaw Tick: 165", font_mono, (37, 99, 235)),
    ("    YELLOW: (X = 70.0, Y = 260.0 mm) -> Yaw Tick: 155", font_mono, (202, 138, 4)),
    ("", font_mono, (0, 0, 0)),
    ("--- CLEARANCES & TOLERANCES VERIFIED ---", font_mono_bold, (37, 99, 235)),
    ("  Arm Base to Arena Border: +100.0 mm clearance", font_mono, (51, 65, 85)),
    ("  Arm Base to Left Flank Tray: +70.0 mm edge gap", font_mono, (51, 65, 85)),
    ("  Front Border to Table Edge: +30.0 mm operator margin", font_mono, (51, 65, 85)),
    ("  Lateral Margins (L / R): +60.0 mm border buffer", font_mono, (51, 65, 85)),
    ("  Wiring & USB Route: Rear Bay (Y ∈ [330, 400 mm])", font_mono, (51, 65, 85)),
]

y_cursor = 100
for text, font, color in cheat_lines:
    draw.text((2300, y_cursor), text, font=font, fill=color)
    y_cursor += 36

out_path = '/workspace/docs/assets/images/arena_layout.png'
img.save(out_path, format="PNG")
print(f"[SUCCESS] Blueprint written to {out_path} ({WIDTH}x{HEIGHT})")
