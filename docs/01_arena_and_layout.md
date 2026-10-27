# Stepping Stone 1: Arena & Physical Component Placement (V-Configuration & 3D-Printable Workcell)

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, optical fiducials, wiring trunking, and 3D-printable modular base architecture for the dual xArm 1s color-sorting robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is organized on a compact, unified **3D-printed modular base** with an overall footprint of **$560\text{ mm} \text{ (Width)} \times 520\text{ mm} \text{ (Depth)}$**, replacing the earlier elongated rectangular baseboard.

```text
  Y (Depth: 520 mm)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Y = 520 mm)                             │
   │  │             [Overhead Camera Boom Mast Anchor]             │
   │  │                                                            │
   │  │          ┌──────────────────────────────────────┐          │
   │  │          │      CENTRAL ELECTRONICS WEDGE       │          │
   │  │          │   (Jetson Orin / RPi 5 + Buck Hub)   │          │
   │  │          └──────────────┬────────┬──────────────┘          │
   │  │                         │        │                         │
   │  │   [Box 1 Nest]     xArm 1        xArm 2     [Box 2 Nest]   │
   │  │   (Left Flank)  (Angled +35°) (Angled -35°) (Right Flank)  │
   │  │   [ArUco #0]   \                /   [ArUco #1]             │
   │  │                 \              /                           │
   │  │                  \  CHAMFERED /                            │
   │  │                   │ HEXAGONAL │                            │
   │  │                   │   ARENA   │ (Elevated +25 mm)          │
   │  │                   │ (Overlap) │                            │
   │  │                   /           \                            │
   │  │                  /             \                           │
   │  │   [ArUco #2] ───┴───────────────┴─── [ArUco #3]            │
   │  │                                                            │
   │  │ FRONT OF WORKSPACE (Operator Position @ Y = 0 mm)          │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 560 mm)
```

- **Origin $(0, 0)$**: Front-Left corner of the unified base assembly.
- **$+X$**: Extends to the right across the workcell ($0\text{ to }560\text{ mm}$). Workcell midline is at $X = 280\text{ mm}$.
- **$+Y$**: Extends from front to rear ($0\text{ to }520\text{ mm}$).
- **$+Z$**: Extends vertically upward from table surface level ($Z = 0\text{ mm}$).
- **Operator Position**: Located at the FRONT ($Y = 0\text{ mm}$), looking toward $+Y$.

---

## 2. Inward-Angled V-Configuration Manipulator Placement

Instead of mounting the two arms flat along a single collinear axis that created an overly elongated strip, the two xArm 1s manipulators are mounted in an **inward-angled V-configuration** symmetric about the workcell midline ($X = 280\text{ mm}$):

| Manipulator | Center Coordinate $(X, Y)$ | Mounting Elevation | Inward Toe-In Heading ($\theta_{\text{nom}}$) | Operating Role |
| :--- | :---: | :---: | :---: | :--- |
| **xArm 1 (Left Arm)** | $(170\text{ mm}, 360\text{ mm})$ | $Z = 0\text{ mm}$ (Base Plate) | **$+35.0^\circ$** (Angled Inward / SE) | Covers left arena quadrant & shared central overlap zone; drops into Left Flank Box 1 |
| **xArm 2 (Right Arm)** | $(390\text{ mm}, 360\text{ mm})$ | $Z = 0\text{ mm}$ (Base Plate) | **$-35.0^\circ$** (Angled Inward / SW) | Covers right arena quadrant & shared central overlap zone; drops into Right Flank Box 2 |

```text
            Y
            ^
            │             X_midline = 280 mm
            │                     │
   (170, 360) xArm 1              │              xArm 2 (390, 360)
            │      ╲  +35°        │        -35° ╱
            │       ╲             │            ╱
            │        ▼            │           ▼
            │               SHARED OVERLAP
            │                    ZONE
            └─────────────────────┴─────────────────────> X
```

### 2.1 Kinematic Spacing & Overlap Verification
- **Center-to-Center Base Spacing**: $S = 390 - 170 = \mathbf{220\text{ mm}}$ along $X$.
- **Baseplate Dimensions**: $140\text{ mm} \text{ (Width)} \times 154\text{ mm} \text{ (Length)}$ ([`docs/xArm demensions.png`](xArm%20demensions.png)).
- **Clearance Between Arm Bases**: With $220\text{ mm}$ center spacing and $140\text{ mm}$ width:
  $$\text{Gap}_{\text{bases}} = 220 - 140 = \mathbf{80\text{ mm}}$$
  Provides generous clearance preventing shoulder collision while keeping the central triangular bay compact.
- **Radial Reach Envelope per Arm**:
  - Minimum Inner Radius ($R_{\min}$): **$120\text{ mm}$** (prevents claw collision with base screws and link brackets).
  - Maximum Safe Radius ($R_{\max}$): **$320\text{ mm}$** (leaves $24\text{ mm}$ margin before straight-arm singularity at $344\text{ mm}$; preserves elbow torque).
- **Central Cooperative Overlap Zone**:
  - The intersecting sweep of both inward-angled arms creates a generous shared manipulation area ($\approx 200\text{ mm} \text{ wide} \times 160\text{ mm} \text{ deep}$) along the midline.
  - Objects placed in this zone can be picked by either arm or handed off dynamically.

---

## 3. Reconfigured Picking Arena: Chamfered Hexagonal Geometry

### 3.1 Resolving the Narrow-Strip Issue
The earlier $25\text{ cm} \times 70\text{ cm}$ rectangular layout resulted in an unnatural aspect ratio with severe corner dead zones. The reconfigured arena uses a **chamfered hexagonal / tapered polygon** geometry that tightly envelopes the natural radial arcs of both arms:

| Dimension / Feature | Value | Engineering Rationale |
| :--- | :---: | :--- |
| **Arena Rear Width** | **$450\text{ mm}$** | Encloses the outer radial span of both arms at the rear picking line ($Y \approx 320\text{ mm}$). |
| **Arena Front Width** | **$350\text{ mm}$** | Tapers inward toward the front ($Y \approx 50\text{ mm}$), matching arm convergence. |
| **Arena Working Depth** | **$380\text{ mm}$** | Extends from $Y = 40\text{ mm}$ to $Y = 420\text{ mm}$. |
| **Usable Picking Area** | **$\approx 1{,}520\text{ cm}^2$** | $100\%$ within the active kinematic reach envelope of at least one arm. Zero blind spots. |
| **Deck Elevation** | **$+25\text{ mm}$** | Lowers shoulder pitch drop from $65\text{ mm}$ to $40\text{ mm}$, optimizing horizontal grasp approaches. |
| **Perimeter Retaining Curb** | **$12\text{ mm}$ Height** | $4\text{ mm}$ thick continuous perimeter wall preventing cubes from falling off the platform. |
| **Surface Mat** | Silicone Sheet ($\mu \approx 0.8$) | High friction matte silicone prevents cube skidding during high-speed gripper approaches. |

---

## 4. Drop Zone Architecture: Immediate Flank Pockets

### 4.1 Eliminating Large Drop Swings
In the previous collinear layout, the arm had to execute an extreme **$125^\circ$ swing** (total travel of $205^\circ$ out of the servo's $240^\circ$ physical ROM) to reach Northwest and Southwest boxes. 

By taking advantage of the $+35^\circ$ / $-35^\circ$ inward rest orientation, collection boxes are relocated to the **immediate lateral flanks** of each arm base:

```text
    Left Arm Drop Travel:                 Right Arm Drop Travel:
    (Arm 1 @ X=170, Y=360)                (Arm 2 @ X=390, Y=360)
         
         [Box 1 (Flank)]                       [Box 2 (Flank)]
                ▲                                     ▲
                 \  Outward Swing:                     /  Outward Swing:
                  \ ~65°-75°                          / ~65°-75°
                   \                                 /
              [xArm 1 Base]                     [xArm 2 Base]
           (Nominal: +35° In)                (Nominal: -35° In)
         Total Travel: ~105° < 240°         Total Travel: ~105° < 240°
```

| Component | Coordinate Center $(X, Y)$ | Elevation | Angular Azimuth from Arm Base |
| :--- | :---: | :---: | :---: |
| **Box 1 (Dedicated to Arm 1)** | $(50\text{ mm}, 360\text{ mm})$ | $Z = 0\text{ mm}$ (Base Well) | **West / Left Flank** ($\approx -70^\circ$ from nominal heading) |
| **Box 2 (Dedicated to Arm 2)** | $(510\text{ mm}, 360\text{ mm})$ | $Z = 0\text{ mm}$ (Base Well) | **East / Right Flank** ($\approx +70^\circ$ from nominal heading) |

### 4.2 Ergonomic Drop Well Features
- **Nesting Wells**: $5\text{ mm}$ deep recessed pockets with $45^\circ$ lead-in chamfers ensure positive mechanical retention without sliding.
- **Finger Cutouts**: Dual semi-circular relief cutouts allow fast, tool-free container removal by an operator.
- **Cycle Time Reduction**: Drop motion angular distance is cut by **$>50\%$**, drastically lowering wear on Motor 6 (base yaw) and increasing sorting throughput.

---

## 5. Central Rear Wedge Electronics & Wiring Bay

In the V-configuration, the space immediately behind and between the two arm bases forms a natural triangular wedge ($X \in [180, 380\text{ mm}], Y \in [380, 510\text{ mm}]$). This is designated as the **Central Electronics & Wiring Bay**:

```text
┌─────────────────────────────────────────────────────────────┐
│                 CENTRAL REAR WEDGE BAY                      │
│               (X: 180-380 mm, Y: 380-510 mm)                │
├─────────────────────────────────────────────────────────────┤
│  1. Single-Board Computer (SBC)                             │
│     - NVIDIA Jetson Orin Nano / Raspberry Pi 5              │
│     - Mounted on M2.5 brass heat-set standoffs              │
│     - Direct USB serial runs to Arm 1 & Arm 2 (< 18 cm)     │
├─────────────────────────────────────────────────────────────┤
│  2. Dual 7.4V / 10A Power Distribution Hub                  │
│     - 2× Independent Buck Regulators (7.4V @ 5A continuous) │
│     - Common ground star-point connection                   │
│     - 1000 µF low-ESR buffer capacitors per arm rail         │
├─────────────────────────────────────────────────────────────┤
│  3. Molded Under-Deck Cable Trunking                        │
│     - Left & Right recessed wiring channels (12 mm x 8 mm)  │
│     - Internal routing eliminates snagging during rotation   │
│     - Rear vertical conduit to overhead camera mast         │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. Optical Framing & Raised ArUco Fiducials

### 6.1 Overhead Camera Mounting
- **Boom Mast Anchor**: Rigid socket molded directly into Tile 2 at the rear apex $(X = 280\text{ mm}, Y = 510\text{ mm})$.
- **Camera Elevation**: Centered over the arena at $(X = 280\text{ mm}, Y = 230\text{ mm})$ at a height of **$65\text{--}75\text{ cm}$** above the base.
- **Field of View (FOV)**: Comfortably encloses the tapered arena and the 4 boundary fiducials without capturing operator movement.

### 6.2 Raised Perimeter Curb ArUco Pads (Hybrid Pocket Method)
To prevent thresholding degradation from filament glare and rounded FDM corners, the 4 ArUco fiducials utilize the **Hybrid Pocket Method**:

```text
                     [3D Printed Pad Cross-Section]
         ┌──────────────────────────────────────────────────┐
  Curb ──┤   ┌──────────────────────────────────────────┐   ├── Curb
 (+12mm) │   │ 0.8 mm Recessed Pocket for Matte Sticker │   │ (+12mm)
         └───┴──────────────────────────────────────────┴───┘
```

1. **Integrated Molded Pads**:
   - 4 fiducial pads are molded directly into the corner vertices of the elevated retaining curb ($Z = +25\text{ mm}$ deck $+ 12\text{ mm} = \mathbf{+37\text{ mm}}$).
   - Each pad contains a **$42\text{ mm} \times 42\text{ mm} \times 0.8\text{ mm}$ recessed pocket**.
2. **Matte Printed Stickers**:
   - High-contrast $40\text{ mm} \times 40\text{ mm}$ ArUco markers printed at 600 DPI on **matte vinyl/paper sticker sheets** press-fit into the pockets.
   - Eliminates specular LED glare, ensures razor-sharp corner boundaries, and guarantees true CAD ground-truth coordinates:

| Marker ID | Role / Location | Global Coordinate $(X, Y, Z)$ |
| :---: | :--- | :---: |
| **ArUco #0** | Rear-Left Perimeter Pad | $(80.0\text{ mm}, 420.0\text{ mm}, 37.0\text{ mm})$ |
| **ArUco #1** | Rear-Right Perimeter Pad | $(480.0\text{ mm}, 420.0\text{ mm}, 37.0\text{ mm})$ |
| **ArUco #2** | Front-Left Perimeter Pad | $(105.0\text{ mm}, 40.0\text{ mm}, 37.0\text{ mm})$ |
| **ArUco #3** | Front-Right Perimeter Pad | $(455.0\text{ mm}, 40.0\text{ mm}, 37.0\text{ mm})$ |

---

## 7. 4-Tile Modular 3D-Printable Base Architecture

To enable fabrication on standard consumer 3D printers ($\le 250\text{ mm} \times 250\text{ mm}$ bed size such as Bambu Lab X1/P1/A1, Creality K1/Ender 3, or Prusa MK3/MK4), the workcell is segmented into **4 interlocking tiles**:

```text
       0 mm                   280 mm                  560 mm
520 mm ┌───────────────────────┬───────────────────────┐ 520 mm
       │        TILE 1         │        TILE 2         │
       │   • Left Arm Base     │   • Right Arm Base    │
       │   • Box 1 Nesting Well│   • Box 2 Nesting Well│
       │   • Left Electronics  │   • Camera Boom Mast  │
       │     Wedge Half        │   • Right Wedge Half  │
260 mm ├═══════════════════════╬═══════════════════════┤ 260 mm
       │        TILE 3         │        TILE 4         │
       │   • Left Arena Deck   │   • Right Arena Deck  │
       │   • Left Curb & Pads  │   • Right Curb & Pads │
       │     (ArUco #0, #2)    │     (ArUco #1, #3)    │
  0 mm └───────────────────────┴───────────────────────┘ 0 mm
       0 mm                   280 mm                  560 mm
```

### 7.1 Interlocking Joints & Structural Assembly
- **Alignment**: Precision interlocking **dovetail / jigsaw tabs** along the interior seam with a $0.20\text{ mm}$ horizontal expansion clearance.
- **Under-Deck Clamping**: 8× under-deck joining plates bolted with **M4 $\times$ 12 mm socket head screws and captive M4 hex nuts** ensure the base acts as a monolithic, rigid platform.
- **Fastener Inserts**:
  - Arm Mounting: 8× **M3 / M4 brass threaded heat-set inserts** melted into the arm mounting bosses.
  - Electronics Bay: 4× **M2.5 brass heat-set standoffs** for the Jetson / RPi.
  - Camera Mast Anchor: 4× **M4 heat-set inserts** for the rigid vertical socket.

### 7.2 Slicing & Manufacturing Guidelines
- **Material**: PLA+ / PETG (Matte finish preferred for non-glare vision background).
- **Perimeters (Walls)**: 5 walls for arm mounting bosses and joint seams; 3 walls for arena floor.
- **Infill**: 35% Gyroid infill on Tile 1 & Tile 2 (torsional arm damping); 20% Grid/Gyroid on Tile 3 & Tile 4.
- **Top/Bottom Layers**: 5 top layers, 4 bottom layers ($0.20\text{ mm}$ layer height).
