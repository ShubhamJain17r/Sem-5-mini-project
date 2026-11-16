# Stepping Stone 1: Arena & Physical Component Placement (Single xArm 1S Setup)

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, optical fiducials, wiring layout, and minimal 3D-printing architecture for the single xArm 1S color-sorting robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is organized on a single, continuous, store-cut **flat wooden baseboard** ($18\text{ mm}$ or $12\text{ mm}$ MDF / shelving board) with a compact footprint of **$400.0\text{ mm} \text{ (Width)} \times 400.0\text{ mm} \text{ (Depth)}$**:

![Realistic 3D Perspective Render of Single xArm Workcell](assets/images/single_arm_setup.png)

![Single xArm 1S Workcell Engineering Blueprint & Arena Layout](assets/images/arena_layout.png)

```text
  Y (Depth: 400 mm)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Y = 400 mm)                             │
   │  │             [Clamp-On Overhead Camera Boom Arm]            │
   │  │                                                            │
   │  │          ┌──────────────────────────────────────┐          │
   │  │          │    REAR ELECTRONICS BAY (Y=350..400) │          │
   │  │          │   (Jetson Orin / RPi 5 + Buck Hub)   │          │
   │  │          └──────────────────┬───────────────────┘          │
   │  │                             │                              │
   │  │   [Tray 1 2x2]           xArm 1S                           │
   │  │   (Left Flank)        (X=200, Y=280)                       │
   │  │   (X=50, Y=280)           [Base]                           │
   │  │        ▲                    │ +23mm Clearance              │
   │  │  90°   │ ═══════════════════╪═════════════════════════════ │
   │  │  Swing │ [ArUco #0]         │                   [ArUco #1] │
   │  │        ▼   (70, 180)        │                   (330, 180) │
   │  │          ┌──────────────────┴────────────────────┐         │
   │  │          │                                       │         │
   │  │          │         ACTIVE PICKING ARENA          │         │
   │  │          │        260 mm (W) x 140 mm (D)        │         │
   │  │          │     X ∈ [70, 330], Y ∈ [40, 180]      │         │
   │  │          │       (Flat Tabletop Z = 0.0 mm)      │         │
   │  │          │                                       │         │
   │  │          │ [ArUco #2]                 [ArUco #3] │         │
   │  │          │ (70, 40)                   (330, 40)  │         │
   │  │          └───────────────────────────────────────┘         │
   │  │                                                            │
   │  │ FRONT OF WORKSPACE (Operator Position @ Y = 0 mm)          │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 400 mm)
```

- **Origin $(0, 0)$**: Front-Left corner of the wooden baseboard.
- **$+X$**: Extends to the right across the workcell ($0\text{ to }400\text{ mm}$). Workcell center is at $X = 200.0\text{ mm}$.
- **$+Y$**: Extends from front to rear ($0\text{ to }400\text{ mm}$).
- **$+Z$**: Extends vertically upward from baseboard top face ($Z = 0.0\text{ mm}$).
- **Table Underside**: $Z = -18.0\text{ mm}$ (for an $18\text{ mm}$ MDF sheet).
- **Operator Position**: Located at the FRONT ($Y = 0\text{ mm}$), looking toward $+Y$.

---

## 2. Central Manipulator Placement

The single xArm 1S manipulator is mounted squarely in the rear-center of the baseboard, facing directly toward the picking arena:

![Single xArm 1S Workcell Engineering Blueprint & Arena Layout](assets/images/arena_layout.png)

| Manipulator | Base Center $(X, Y)$ | Mounting Elevation | Nominal Heading ($\theta_{\text{nom}}$) | Operating Role |
| :--- | :---: | :---: | :---: | :--- |
| **xArm 1S (Central)** | $(200.0\text{ mm}, 280.0\text{ mm})$ | $Z = 0.0\text{ mm}$ (Direct Flat Mount) | **$-90.0^\circ$** (South, facing $-Y$) | Sorts all objects in the active arena into the Left Flank 2×2 tray |

### 2.1 Kinematic Dimensions & Reach Verification
- **Base Center Position**: $(X = 200.0\text{ mm}, Y = 280.0\text{ mm})$.
- **Baseplate Dimensions**: $140.0\text{ mm} \text{ (Width)} \times 154.0\text{ mm} \text{ (Length)}$ ([`docs/assets/images/xArm demensions.png`](assets/images/xArm%20demensions.png)).
- **Clearance to Arena**: With arm center at $Y = 280.0\text{ mm}$, front of baseplate is at $Y = 203.0\text{ mm}$. With the arena rear boundary at $Y = 180.0\text{ mm}$, there is a clear $+23.0\text{ mm}$ physical buffer between base metal and foam border.
- **Radial Reach Envelope**:
  - Minimum Inner Radius ($R_{\min}$): **$130.0\text{ mm}$** (prevents claw collision with base screws and link brackets).
  - Maximum Safe Radius ($R_{\max}$): **$280.0\text{ mm}$** (leaves margin before singularity at $344\text{ mm}$; preserves elbow torque).
- **Arena Coverage**:
  - Closest arena point: $(200.0, 180.0)$ $\to$ $r = 100.0\text{ mm}$ (reachable with tucked wrist).
  - Furthest arena corners: $(70.0, 40.0)$ and $(330.0, 40.0)$ $\to$ $dx = 130.0\text{ mm}$, $dy = 240.0\text{ mm}$, $r = \sqrt{130^2 + 240^2} \approx 272.9\text{ mm} \le 280.0\text{ mm}$ ($R_{\max}$).
  - 100% of the active arena lies within the reachable envelope.

---

## 3. Flat Rectangular Picking Arena

### 3.1 Resolving Complex Woodworking and Riser Issues
In this design, the picking arena requires **zero woodworking cuts and zero elevated deck risers**:
- The arena floor sits directly flat on the wooden baseboard at **$Z = 0.0\text{ mm}$**.
- The boundary is bounded by a **$260.0\text{ mm} \times 140.0\text{ mm}$** rectangle centered along $X = 200.0\text{ mm}$:
  - Left boundary: $X = 70.0\text{ mm}$
  - Right boundary: $X = 330.0\text{ mm}$
  - Front boundary: $Y = 40.0\text{ mm}$
  - Rear boundary: $Y = 180.0\text{ mm}$
- **Perimeter Retaining Border**: Formed by standard **self-adhesive foam weatherstripping tape ($8\text{--}10\text{ mm}$ tall $\times 10\text{--}12\text{ mm}$ wide)** stuck directly along the perimeter.
  - Soft compliance prevents any servo overload if a claw finger nudges the border.
  - Keeps wooden craft cubes from skidding or rolling off the active zone.
- **Target Sorting Blocks**: $1.25" \times 1.25" \times 0.75"$ ($31.75\text{ mm} \times 31.75\text{ mm} \times 19.05\text{ mm}$) hardwood craft blocks.
  - Grasp target center is at **$Z_{\text{target}} = 9.53\text{ mm}$** ($0.0\text{ mm} + 19.05/2\text{ mm}$), configured in software with zero physical risers.
- *(Optional)* High-friction matte silicone mat ($\mu \approx 0.8$, $1.0\text{ mm}$ thickness) placed inside the arena to ensure zero cube skidding during high-speed gripper closure.

---

## 4. Drop Zone Architecture: Immediate Left Flank Single Open Container

### 4.1 Single Large Collection Container
To eliminate jamming of the $31.75\text{ mm}$ square blocks (which have a $44.9\text{ mm}$ diagonal profile), the collection zone uses a **single large open container** without internal divider partitions:
- Outer Dimensions: $\approx 100\text{ mm} \times 100\text{ mm}$, rim height $35\text{ mm}$.
- Avoids dividing walls so blocks drop cleanly at any yaw angle.

### 4.2 Drop Coordinates & Compact Swings
By placing the single collection container on the **immediate Left Flank** of the arm base:
- **Container Center**: $(X = 50.0\text{ mm}, Y = 280.0\text{ mm}, Z = 0.0\text{ mm})$.
- **Radial Reach**: $r = 150.0\text{ mm}$ from base center $(200, 280)$.
- **Angular Travel**: Clean $90^\circ$ clockwise swing from the south arena heading (nominal base yaw tick $\approx 145$).
- **Drop Target Coordinate**: $(X = 50.0\text{ mm}, Y = 280.0\text{ mm}, Z = 40.0\text{ mm})$ for all sorted blocks.

---

## 5. Optical Framing & Planar ArUco Fiducials

### 5.1 Overhead Camera & Lighting Mounting
- **Freestanding Desk Clamp / Boom Stand**: Mounted separately from the $400\times 400\text{ mm}$ wooden baseboard (isolates arm vibration from the camera sensor):
  - **Mount Height ($H$)**: **$450\text{--}500\text{ mm}$** above baseboard surface.
  - **Center Alignment**: Reaches directly over $(X = 200.0\text{ mm}, Y = 110.0\text{ mm})$ oriented nadir downward.
  - **Pixel Density**: Delivers $> 3.0\text{ px/mm}$ metric resolution across the active arena while preserving $> 100\text{ mm}$ vertical clearance over the robot arm.
- **Lighting**: Dimmable LED ring light mounted concentrically around camera lens facing nadir downward.

### 5.2 Coplanar Board-Mounted ArUco Fiducials
4 ArUco markers (`DICT_4X4_50`, IDs 0, 1, 2, 3) printed at $30.0\text{ mm} \times 30.0\text{ mm}$ on matte vinyl sticker paper are applied directly flat to the wooden board at the four corners of the arena:

| Marker ID | Placement Role | World Coordinate $(X, Y, Z)$ | Pixel Destination $(u, v)$ |
| :---: | :--- | :---: | :---: |
| **ArUco #0** | Rear-Left Arena Corner | $(70.0\text{ mm}, 180.0\text{ mm}, 0.0\text{ mm})$ | $(0, 0)$ |
| **ArUco #1** | Rear-Right Arena Corner | $(330.0\text{ mm}, 180.0\text{ mm}, 0.0\text{ mm})$ | $(520, 0)$ |
| **ArUco #2** | Front-Left Arena Corner | $(70.0\text{ mm}, 40.0\text{ mm}, 0.0\text{ mm})$ | $(0, 280)$ |
| **ArUco #3** | Front-Right Arena Corner | $(330.0\text{ mm}, 40.0\text{ mm}, 0.0\text{ mm})$ | $(520, 280)$ |

> [!NOTE]
> **Planar Homography Accuracy**: Because all four ArUco markers and the arena floor are strictly coplanar at **$Z = 0.0\text{ mm}$**, OpenCV planar homography produces a mathematically perfect perspective warp ($H$) into a rectified $520 \times 280\text{ px}$ grid at $2.0\text{ px/mm}$ with zero elevation-induced parallax error!

---

## 6. Electronics & Cable Management

- **Single-Board Computer**: NVIDIA Jetson Orin Nano or Raspberry Pi 5 mounted on the rear section of the board ($Y \approx 350\text{--}400\text{ mm}$) using 4× adhesive nylon PCB standoffs.
- **Power Distribution Hub**: Single 7.4V / 5A DC buck regulator mounted beside the SBC with short USB serial leads and thick 18 AWG power wiring.
- **Wiring Routing**: Flat surface cable clips; zero need for complex under-deck CNC routing or molded conduit channels.
