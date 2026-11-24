# Stepping Stone 1: Arena & Physical Component Placement (Single xArm 1S Setup)

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, optical fiducials, wiring layout, and workspace architecture for the single xArm 1S dual-color (Red & Green) sorting robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is organized on a single, continuous **flat wooden baseboard** with a drawn **$7 \times 12$ grid** ($5\text{ cm} \times 5\text{ cm}$ square cells) giving a workspace footprint of **$350.0\text{ mm} \text{ (Width)} \times 600.0\text{ mm} \text{ (Depth)}$**:

```text
  Y (Depth: 600 mm, Rows 11 to 0)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Row 0, Y = 600 mm)                      │
   │  │ ┌───────────────────────┐        ┌───────────────────────┐ │
   │  │ │   RED CONTAINER BOX   │        │  GREEN CONTAINER BOX  │ │
   │  │ │  (Rows 0..1, Cols 0..1)        │  (Rows 0..1, Cols 5..6) │
   │  │ │ Drop Yaw: 650 ticks   │        │ Drop Yaw: 354 ticks   │ │
   │  │ └───────────────────────┘        └───────────────────────┘ │
   │  │   [ArUco #0] (50x50mm)      ┌───────────────────┐ [ArUco #1] (50x50mm) │
   │  │   (25, 425) [Row 3, Col 0]  │   xArm 1S MOUNT   │ (325, 425) [Row 3, C6]│
   │  │                             │ (X=175.0, Y=375.0)│                       │
   │  │   EXTENDED PICKING ARENA    │ Heading: -90° (S) │ EXTENDED PICKING ARENA│
   │  │   (Rows 3..6, Cols 0, 1)    │ (Rows 3..5, C2..4)│ (Rows 3..6, Cols 5, 6)│
   │  │                             └───────────────────┘                       │
   │  │                                                                        │
   │  │               EXTENDED FULL PICKING ARENA (Rows 7..10)                  │
   │  │                     (r = 124.5..335.5 mm reach, 45 Cells)               │
   │  │             [Red & Green Targets | Nearest-First Sorting]               │
   │  │                                                                        │
   │  │   [ArUco #2] (50x50mm)                            [ArUco #3] (50x50mm) │
   │  │   (25, 25) [Row 11, Col 0]                        (325, 25) [Row 11, C6]│
   │  │                                                                        │
   │  │ FRONT OF WORKSPACE (Operator Position @ Row 11, Y = 0 mm)              │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 350 mm, Cols 0 to 6)
```

- **Origin $(0, 0)$**: Front-Left corner of the platform (Row 11, Column 0).
- **$+X$**: Left to right across columns 0 to 6 ($0\text{ to }350\text{ mm}$). Workcell center is at $X = 175.0\text{ mm}$.
- **$+Y$**: Front to rear across rows 11 to 0 ($0\text{ to }600\text{ mm}$).
- **$+Z$**: Extends vertically upward from platform top face ($Z = 0.0\text{ mm}$).
- **ArUco Markers**: $50\text{ mm} \times 50\text{ mm}$ markers in cells (3,0), (3,6), (11,0), (11,6) (DICT_4X4_50). Objects are strictly excluded from ArUco cells.

---

## 2. Central Manipulator Placement

The single xArm 1S manipulator is mounted in Rows 3..5, Columns 2..4:

| Manipulator | Base Center $(X, Y)$ | Mounting Elevation | Nominal Heading ($\theta_{\text{nom}}$) | Operating Role |
| :--- | :---: | :---: | :---: | :--- |
| **xArm 1S (Central)** | $(175.0\text{ mm}, 375.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | **$-90.0^\circ$** (South down Col 3) | Picks Red & Green cubes from arena; deposits into rear containers via overhead rear-flip |

### 2.1 Kinematic Reach & Extended 45-Cell Grid Verification
- **Base Center Position**: $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$.
- **Baseplate Dimensions**: $140.0\text{ mm} \times 154.0\text{ mm}$ within $150\times 150\text{ mm}$ exclusion zone (Cols 2..4, Rows 3..5).
- **Radial Reach Envelope**: $R \in [124.5, 335.5\text{ mm}]$.
- **45 Valid Picking Cells (Rows 3..10)**:
  - Row 3: Cols 1, 5 (2 cells; ArUco 0 & 1 and xArm base excluded)
  - Row 4: Cols 0, 1, 5, 6 (4 cells; xArm base excluded)
  - Row 5: Cols 0, 1, 5, 6 (4 cells; xArm base excluded)
  - Row 6: Cols 0..6 (7 cells; near-pedestal offsets applied)
  - Rows 7, 8, 9, 10: Cols 0..6 each (28 cells)
  - Total: **45 picking cells (100% reachable with analytical IK)**.
- **Red Container Reach**: $(X = 50.0, Y = 550.0) \to R = 215.1\text{ mm}$, rear-flip drop yaw tick $650$.
- **Green Container Reach**: $(X = 300.0, Y = 550.0) \to R = 215.1\text{ mm}$, rear-flip drop yaw tick $354$.

### 2.2 The Golden Pickup Zone (Kinematic Sweet Spot)
While the extended arena encompasses Rows 3..10, autonomous production picking operates inside the **Golden Pickup Zone**:
- **Grid Location**: Rows 7..9, Columns 2..4 (9 core cells centered down the midline).
- **Cartesian Bounds**: $X \in [75.0, 275.0\text{ mm}]$, $Y \in [100.0, 250.0\text{ mm}]$.
- **Radial Reach Envelope**: $r \in [140.0, 260.0\text{ mm}]$ from xArm base $(175, 375)$.
- **Kinematic Advantages**:
  1. Operates purely within the mid-stroke dexterity region where shoulder ($\theta_5$) and elbow ($\theta_4$) provide maximum torque margin without approaching joint singularities.
  2. Enables pure top-down overhead plunge ($Z = 75\text{ mm} \to Z = 22\text{ mm}$), completely eliminating horizontal nudging or lateral collisions with adjacent blocks.
  3. Seamlessly accommodates rectangular craft blocks ($60\times 35\text{ mm}$) with dynamic wrist roll (Motor 2 at $130\text{ ticks}$ for horizontal, $500\text{ ticks}$ for vertical/square).

---

## 3. Physical Hand-Drawn Grid Arena

- The arena floor has a $7 \times 12$ grid of $5\text{ cm} \times 5\text{ cm}$ squares drawn directly on the board.
- **Perimeter Retaining Border**: Self-adhesive foam tape ($8\text{--}10\text{ mm}$ tall) along arena boundary.
- **Target Objects**: Red and Green colored craft blocks (sorted into respective rear containers). Blue chassis is bypassed and masked.

---

## 4. Drop Zones: Rear Flank Dual Container Boxes

- **Red Container Box**: Placed in Rows 0..1, Columns 0..1 ($100\times 100\text{ mm}$); Center $(X = 50.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, rear drop yaw tick **$650$**.
- **Green Container Box**: Placed in Rows 0..1, Columns 5..6 ($100\times 100\text{ mm}$); Center $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, rear drop yaw tick **$354$**.
- **Symmetric Radial Reach**: Both containers sit at identical radial radius $R = 215.1\text{ mm}$ from base center $(175.0, 375.0\text{ mm})$.

---

## 5. Optical Framing & Planar ArUco Fiducials

### 5.1 Overhead Camera & Lighting Mounting
- **Freestanding Desk Clamp / Boom Stand**: Mounted separately from the platform board (isolates arm vibration from the camera sensor):
  - **Mount Height ($H$)**: **$600\text{--}650\text{ mm}$** above baseboard surface.
  - **Center Alignment**: Reaches directly over $(X = 175.0\text{ mm}, Y = 250.0\text{ mm})$ oriented nadir downward.
  - **Pixel Density**: Delivers $\ge 2.0\text{ px/mm}$ metric resolution across the active arena while preserving $> 150\text{ mm}$ vertical clearance over the robot arm.
- **Lighting**: Dimmable LED ring light mounted concentrically around camera lens facing nadir downward.

### 5.2 Coplanar Board-Mounted ArUco Fiducials
4 ArUco markers (`DICT_4X4_50`, IDs 0, 1, 2, 3) printed at $50.0\text{ mm} \times 50.0\text{ mm}$ on matte vinyl sticker paper are applied directly flat to the platform board at marker cells:

| Marker ID | Placement Role | World Coordinate $(X, Y, Z)$ | Pixel Destination $(u, v)$ |
| :---: | :--- | :---: | :---: |
| **ArUco #0** | Row 3, Col 0 (Top-Left) | $(25.0\text{ mm}, 425.0\text{ mm}, 0.0\text{ mm})$ | $(0, 0)$ |
| **ArUco #1** | Row 3, Col 6 (Top-Right) | $(325.0\text{ mm}, 425.0\text{ mm}, 0.0\text{ mm})$ | $(600, 0)$ |
| **ArUco #2** | Row 11, Col 0 (Bottom-Left) | $(25.0\text{ mm}, 25.0\text{ mm}, 0.0\text{ mm})$ | $(0, 800)$ |
| **ArUco #3** | Row 11, Col 6 (Bottom-Right) | $(325.0\text{ mm}, 25.0\text{ mm}, 0.0\text{ mm})$ | $(600, 800)$ |

> [!NOTE]
> **Planar Homography Accuracy**: Because all four ArUco markers and the arena floor are strictly coplanar at **$Z = 0.0\text{ mm}$**, OpenCV planar homography produces a mathematically perfect perspective warp ($H$) into a rectified $600 \times 800\text{ px}$ grid at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$) with zero elevation-induced parallax error!

---

## 6. Electronics & Cable Management

- **Single-Board Computer**: NVIDIA Jetson Orin Nano or Raspberry Pi 5 mounted on the rear section of the board ($Y \approx 500\text{--}600\text{ mm}$) using 4× adhesive nylon PCB standoffs.
- **Power Distribution Hub**: Single 7.4V / 5A DC buck regulator mounted beside the SBC with short USB serial leads and thick 18 AWG power wiring.
- **Wiring Routing**: Flat surface cable clips; zero need for complex under-deck CNC routing or molded conduit channels.
