# Stepping Stone 1: Arena & Physical Component Placement (Single xArm 1S Setup)

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, optical fiducials, wiring layout, and workspace architecture for the single xArm 1S red-object picking robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is organized on a single, continuous **flat wooden baseboard** with a drawn **$7 \times 12$ grid** ($5\text{ cm} \times 5\text{ cm}$ square cells) giving a workspace footprint of **$350.0\text{ mm} \text{ (Width)} \times 600.0\text{ mm} \text{ (Depth)}$**:

```text
  Y (Depth: 600 mm, Rows 11 to 0)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Row 0, Y = 600 mm)                      │
   │  │                                  ┌───────────────────────┐ │
   │  │                                  │ CONTAINER BOX (100x100│ │ (X=300, Y=550)
   │  │                                  │ Rows 0..1, Cols 5..6) │ │ Drop Yaw: 875 ticks
   │  │                                  └───────────────────────┘ │
   │  │   [ArUco #0] (50x50mm)      ┌───────────────────┐ [ArUco #1│
   │  │   (25, 425) [Row 3, Col 0]  │   xArm 1S MOUNT   │ (325, 425│
   │  │                             │ (X=175.0, Y=375.0)│ [Row 3, C│
   │  │                             │ Heading: -35.0 deg│          │
   │  │                             └───────────────────┘          │
   │  │                                                            │
   │  │               ACTIVE PICKING ARENA (Rows 7..10)            │
   │  │                     (r = 160..300 mm reach)                │
   │  │               [Red = Target | Green = Distractor]          │
   │  │                                                            │
   │  │   [ArUco #3] (50x50mm)                            [ArUco #2│
   │  │   (25, 25) [Row 11, Col 0]                        (325, 25)│
   │  │                                                   [Row 11, │
   │  │ FRONT OF WORKSPACE (Operator Position @ Row 11, Y = 0 mm)  │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 350 mm, Cols 0 to 6)
```

- **Origin $(0, 0)$**: Front-Left corner of the platform (Row 11, Column 0).
- **$+X$**: Left to right across columns 0 to 6 ($0\text{ to }350\text{ mm}$). Workcell center is at $X = 175.0\text{ mm}$.
- **$+Y$**: Front to rear across rows 11 to 0 ($0\text{ to }600\text{ mm}$).
- **$+Z$**: Extends vertically upward from platform top face ($Z = 0.0\text{ mm}$).
- **ArUco Markers**: $50\text{ mm} \times 50\text{ mm}$ markers in cells (3,0), (3,6), (11,6), (11,0).

---

## 2. Central Manipulator Placement

The single xArm 1S manipulator is mounted in Rows 3..5, Columns 2..4:

| Manipulator | Base Center $(X, Y)$ | Mounting Elevation | Nominal Heading ($\theta_{\text{nom}}$) | Operating Role |
| :--- | :---: | :---: | :---: | :--- |
| **xArm 1S (Central)** | $(175.0\text{ mm}, 375.0\text{ mm})$ | $Z = 0.0\text{ mm}$ | **$-35.0^\circ$** (South-East) | Picks red objects from active arena and places into Rear Flank container |

### 2.1 Kinematic Reach Verification
- **Base Center Position**: $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$.
- **Baseplate Dimensions**: $140.0\text{ mm} \times 154.0\text{ mm}$ within $150\times 150\text{ mm}$ zone (Cols 2..4, Rows 3..5).
- **Radial Reach Envelope**: $R \in [160, 300\text{ mm}]$.
- **Picking Coverage**: Rows 7..10 ($Y \in [50, 250\text{ mm}]$) are 100% reachable.
- **Container Reach**: $(X = 300.0, Y = 550.0) \to r = 215.1\text{ mm}$, yaw tick $875$ (safe $\le 879$).

---

## 3. Physical Hand-Drawn Grid Arena

- The arena floor has a $7 \times 12$ grid of $5\text{ cm} \times 5\text{ cm}$ squares drawn directly on the board.
- **Perimeter Retaining Border**: Self-adhesive foam tape ($8\text{--}10\text{ mm}$ tall) along arena boundary.
- **Target Objects**: Red colored craft blocks (picked). Green blocks present as distractors (ignored). Blue chassis is bypassed and masked.

---

## 4. Drop Zone: Rear Flank Single Container Box

- Physical container box placed in Rows 0..1, Columns 5..6 ($100\times 100\text{ mm}$).
- **Container Center**: $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 0.0\text{ mm})$.
- **Radial Reach**: $r = 215.1\text{ mm}$ from base center $(175, 375)$.
- **Drop Target Coordinate**: $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 40.0\text{ mm})$, yaw tick $875$.

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
| **ArUco #0** | Row 3, Col 0 Corner | $(25.0\text{ mm}, 425.0\text{ mm}, 0.0\text{ mm})$ | $(0, 0)$ |
| **ArUco #1** | Row 3, Col 6 Corner | $(325.0\text{ mm}, 425.0\text{ mm}, 0.0\text{ mm})$ | $(600, 0)$ |
| **ArUco #2** | Row 11, Col 6 Corner | $(325.0\text{ mm}, 25.0\text{ mm}, 0.0\text{ mm})$ | $(600, 800)$ |
| **ArUco #3** | Row 11, Col 0 Corner | $(25.0\text{ mm}, 25.0\text{ mm}, 0.0\text{ mm})$ | $(0, 800)$ |

> [!NOTE]
> **Planar Homography Accuracy**: Because all four ArUco markers and the arena floor are strictly coplanar at **$Z = 0.0\text{ mm}$**, OpenCV planar homography produces a mathematically perfect perspective warp ($H$) into a rectified $600 \times 800\text{ px}$ grid at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$) with zero elevation-induced parallax error!

---

## 6. Electronics & Cable Management

- **Single-Board Computer**: NVIDIA Jetson Orin Nano or Raspberry Pi 5 mounted on the rear section of the board ($Y \approx 500\text{--}600\text{ mm}$) using 4× adhesive nylon PCB standoffs.
- **Power Distribution Hub**: Single 7.4V / 5A DC buck regulator mounted beside the SBC with short USB serial leads and thick 18 AWG power wiring.
- **Wiring Routing**: Flat surface cable clips; zero need for complex under-deck CNC routing or molded conduit channels.
