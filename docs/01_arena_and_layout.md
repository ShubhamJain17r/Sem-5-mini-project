# Stepping Stone 1: Arena & Physical Component Placement

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, lighting, wiring, and electrical layout for the dual xArm 1s color-sorting robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is constructed on a single rigid wooden or aluminum baseboard measuring **$800\text{ mm} \text{ (Width)} \times 600\text{ mm} \text{ (Depth)}$**.

```
  Y (Depth: 600 mm)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ BACK OF WORKSPACE (600 mm)                                 │
   │  │                                                            │
   │  │   [Box 1 - NW]          xArm 1 (Back @ 420mm)              │
   │  │                                        \                   │
   │  │                                         \ CONTINUOUS       │
   │  │   [Central Electronics   Collinear Axis  │ STADIUM ARENA   │
   │  │    Bay: Jetson & Hub]    (X = 360 mm)    │ (Elevated 15mm) │
   │  │                                         /                  │
   │  │   [Box 2 - SW]          xArm 2 (Front @ 180mm)             │
   │  │                                                            │
   │  │ FRONT OF WORKSPACE (Operator Position @ Y = 0 mm)          │
   │  └────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 800 mm)
```

- **Origin $(0, 0)$**: Front-Left corner of the baseboard.
- **$+X$**: Extends to the right toward the picking arena ($0\text{ to }800\text{ mm}$).
- **$+Y$**: Extends from front to back ($0\text{ to }600\text{ mm}$).
- **$+Z$**: Extends vertically upward from baseboard level ($Z = 0\text{ mm}$).
- **Operator Position**: Located at the FRONT ($Y = 0\text{ mm}$), looking toward $+Y$.

---

## 2. Robotic Manipulator Placement (Collinear Inner Axis)

The two xArm 1s manipulators are mounted along a single shared physical line of symmetry at **$X = 360\text{ mm}$**:

| Manipulator | Center Coordinate $(X, Y)$ | Mounting Elevation | Operating Role |
|---|:---:|:---:|---|
| **xArm 2 (Front Arm)** | $(360\text{ mm}, 180\text{ mm})$ | $Z = 0\text{ mm}$ (Flat on Table) | Covers front arena sector & drops into South-West Box 2 |
| **xArm 1 (Back Arm)** | $(360\text{ mm}, 420\text{ mm})$ | $Z = 0\text{ mm}$ (Flat on Table) | Covers back arena sector & drops into North-West Box 1 |

### Kinematic Spacing Verification:
- **Center-to-Center Spacing**: $S = 420 - 180 = \mathbf{240\text{ mm}}$.
- **Baseplate Air Gap**: With baseplate dimensions of $154\text{ mm} \times 140\text{ mm}$ ([`docs/xArm demensions.png`](file:///d:/Sem-3-mini-project/docs/xArm%20demensions.png)), the clear gap between bases is:
  $$\text{Gap}_{\text{bases}} = 240 - 154 = \mathbf{86\text{ mm}}$$
- **Table Margins**: Front and back margins are symmetrically identical at $180\text{ mm}$ ($103\text{ mm}$ clear from baseplate edge to table edge), leaving ample space for ArUco markers and boom clamping.
- **Midline Reach**: Midline partition is at $Y = 300\text{ mm}$ ($120\text{ mm}$ from each arm). A cube at the center of the arena ($X = 150\text{ mm}$) requires:
  $$R = \sqrt{150^2 + 120^2} = 192.1\text{ mm}$$
  Both arms easily reach across the midline without exceeding their maximum bounded reach of $R_{\max} = 240\text{ mm}$.

---

## 3. Drop Zone Architecture: Solving the $240^\circ$ Base Servo ROM

### 3.1 The Hardware Constraint:
The **LewanSoul / Hiwonder LX-15D** bus servos driving the arm base yaw have a maximum mechanical travel range of **$240^\circ$** ($0\text{ to }1000$ tick range).
- If the servo's mechanical center ($120^\circ$) points East toward the picking arena ($0^\circ$), it can only swing:
  - Counter-Clockwise (CCW): up to $+120^\circ$ (North-West)
  - Clockwise (CW): down to $-120^\circ$ (South-West)
- **Due West ($180^\circ$, directly behind the arm) is a physical deadzone.** Attempting a $180^\circ$ drop swing causes servo stall or internal gear damage.

### 3.2 The Angled NW / SW Drop Box Solution:
By moving the collection boxes to the **North-West (NW)** and **South-West (SW)** corners of each arm's reachable arc, the drop motion remains well within the $240^\circ$ physical limit:

```
  North-West Drop:                  South-West Drop:
  (Arm 1 @ Y = 420 mm)              (Arm 2 @ Y = 180 mm)
         ^                                 |
         │ \ Angle: +125°                  │ \ Angle: -125°
         │  \                              │  \
         │   ▼                             │   ▼
     [Box 1 (NW)]                      [Box 2 (SW)]
  Total Travel: 205° <= 240°        Total Travel: 205° <= 240°
```

| Component | Coordinate Center $(X, Y)$ | Elevation | Angular Orientation from Arm Base |
|---|:---:|:---:|:---:|
| **Box 1 (Dedicated to Arm 1)** | $(115\text{ mm}, 500\text{ mm})$ | $Z = 0\text{ mm}$ | **North-West** ($\approx +125^\circ$ azimuth) |
| **Box 2 (Dedicated to Arm 2)** | $(115\text{ mm}, 100\text{ mm})$ | $Z = 0\text{ mm}$ | **South-West** ($\approx -125^\circ$ azimuth) |
| **Central Electronics Bay** | $(130\text{ mm}, 300\text{ mm})$ | $Z = 0\text{ mm}$ | Centered between boxes on the left flank |

- **Arm 1 Travel Envelope**: Picks between $-75^\circ$ (midline) and $+35^\circ$ (back-right). Swings CCW to $+125^\circ$ for drop. Total span: $35^\circ - (-75^\circ) + 90^\circ = \mathbf{200^\circ} < 240^\circ$.
- **Arm 2 Travel Envelope**: Picks between $-35^\circ$ (front-right) and $+75^\circ$ (midline). Swings CW to $-125^\circ$ for drop. Total span: $75^\circ - (-35^\circ) + 90^\circ = \mathbf{200^\circ} < 240^\circ$.

---

## 4. Central Electronics & Power Bay

Relocating Box 1 to the North-West and Box 2 to the South-West opens up a large, unutilized rectangular area on the left flank ($X \in [50, 210\text{ mm}], Y \in [220, 380\text{ mm}]$).

This area is designated as the **Central Electronics & Wiring Bay**:

```
┌─────────────────────────────────────────────────────────────┐
│                 CENTRAL ELECTRONICS BAY                     │
│                                                             │
│   ┌───────────────────────┐     ┌───────────────────────┐   │
│   │ Jetson Nano / RPi 5   │     │ 7.4V / 10A Power Hub  │   │
│   │ • ROS 2 Compute Node  │     │ • Step-down Regulators│   │
│   │ • USB-UART Ports      │     │ • Common Ground Bus   │   │
│   └───────────┬───────────┘     └───────────┬───────────┘   │
│               │                             │               │
│               ▼                             ▼               │
│      USB Serial to Arm 1            7.4V Power to Arm 1     │
│      USB Serial to Arm 2            7.4V Power to Arm 2     │
│      USB Overhead Camera            12V DC Input Barrel     │
└─────────────────────────────────────────────────────────────┘
```

### Advantages:
1. **Ultra-Short Cable Runs**: USB-serial cables to Arm 1 ($20\text{ cm}$) and Arm 2 ($20\text{ cm}$) minimize electromagnetic interference (EMI) and line resistance.
2. **Camera Boom Alignment**: Sits directly underneath the camera boom stand base at $Y = 300\text{ mm}$.
3. **Thermal Safety**: Away from student hands and arm moving linkages, with room for passive aluminum heatsinks and cooling fans.
4. **Zero Vision Intrusion**: Sits completely outside the camera's warped homography frame.

---

## 5. Elevated Picking Arena Construction (+15 to 20 mm)

### 5.1 Platform Specifications
- **Platform Dimensions**: $425\text{ mm} \text{ (Width)} \times 530\text{ mm} \text{ (Depth)}$ ($X \in [330, 755\text{ mm}], Y \in [35, 565\text{ mm}]$).
- **Core Substrate**: $16\text{ mm}$ or $18\text{ mm}$ high-density MDF or CNC-cut acrylic sheet.
- **Surface Elevation**: $+18\text{ mm} \pm 2\text{ mm}$ above the baseboard.
- **Why this elevation is mandatory**: The xArm 1s shoulder pitch axis (Motor 2) is at $Z = 65\text{ mm}$. Elevating the arena floor brings the center of $25\text{ mm}$ cubes to $Z \approx 30.5\text{ mm}$. The arm reaches horizontally into its highest torque band without the claw scraping the table floor.

### 5.2 Silicone Friction Mat
- **Material**: $1.5\text{--}2.0\text{ mm}$ food-grade silicone baking sheet.
- **Color**: **Matte Neutral 18% Gray** (or matte light cyan). Avoid high-gloss white (causes specular hotspots from ceiling lights) and black (absorbs blue contour edges).
- **Adhesion**: Affixed permanently using **high-tack double-sided acrylic carpet tape** or 3M 77 spray adhesive to prevent wrinkles, ripples, or bunching when the gripper slides forward during radial ingress.

---

## 6. Physical Perimeter Barriers & Boundaries

To prevent cubes from rolling off the platform or into mechanical pinch zones:

| Barrier | Placement Radius | Wall Height | Purpose |
|---|:---:|:---:|---|
| **Outer Sector Boundary** | Curved arc at $R_{\max} = 240\text{ mm}$ from each arm base | $25.0\text{ mm}$ | Prevents cubes from settling outside safe torque reach ($R > 240\text{ mm}$) |
| **Inner Base Shield** | Curved arc at $R_{\min} = 100\text{ mm}$ from each arm base | $20.0\text{ mm}$ | Prevents cubes from entering baseplate self-collision zone ($R < 100\text{ mm}$) |
| **Midline Visual Marker** | Along $Y = 300\text{ mm}$ line from $X = 360$ to $650\text{ mm}$ | Flush (colored tape) | Visual reference for static task split between Arm 1 and Arm 2 |

*Vertical Clearance*: Upper arm links rotate at $Z > 65\text{ mm}$, providing $>40\text{ mm}$ of clear vertical clearance over the $25\text{ mm}$ barrier walls.

---

## 7. Collection Box Geometry (Anti-Bounce & Funneling)

Both dedicated $2\times 2$ collection boxes share identical physical features:
- **Outer Box Dimensions**: $100\text{ mm} \times 100\text{ mm} \times 40\text{ mm}$ (acrylic or 3D-printed PLA).
- **Cell Dimensions**: 4 compartments, each $46\text{ mm} \times 46\text{ mm}$ (holds up to 4–6 cubes each).
- **Chamfered / Knife-Edge Dividers**: The internal cross-dividers feature $45^\circ$ triangular beveled knife-edges. If an arm drops a cube slightly off-center due to gear backlash, the beveled edge funnels the cube cleanly into the compartment rather than letting it balance on the rim.
- **Acoustic / Impact Damping Floor**: Each compartment bottom is lined with a $2\text{ mm}$ square of EVA foam or craft felt to absorb impact energy and prevent cubes from bouncing into adjacent cells.

---

## 8. Overhead Camera & Illumination Rig

- **Camera Boom**: Heavy-duty vertical desktop boom stand clamped to table edge at $(X = 540\text{ mm}, Y = 300\text{ mm})$ or on the left border.
- **Camera Height**: Fixed at **$65\text{--}75\text{ cm}$** directly over the geometric center of the elevated arena $(X = 540\text{ mm}, Y = 300\text{ mm})$.
- **Dedicated Diffuse Lighting**: A $6\text{-inch}$ USB-powered diffuse LED ring light is mounted coaxially around the camera lens.
- **Optical Invariance**: Camera software locks **Manual Exposure** and **Fixed White Balance (4500K)**, eliminating HSV threshold drift across morning, afternoon, and artificial room lighting.

---

## 9. ArUco Marker Mounting & Calibration

4 ArUco markers (Dictionary: `cv2.aruco.DICT_4X4_50`) frame the elevated picking arena:

| Marker ID | Placement | Nominal $(X, Y)$ Center | Elevation |
|:---:|---|:---:|:---:|
| **ArUco #0** | Inner Front (Collinear with arms) | $(360\text{ mm}, 60\text{ mm})$ | **$Z = +18\text{ mm}$** (Elevated Platform) |
| **ArUco #1** | Outer Front | $(720\text{ mm}, 60\text{ mm})$ | **$Z = +18\text{ mm}$** (Elevated Platform) |
| **ArUco #2** | Inner Back (Collinear with arms) | $(360\text{ mm}, 540\text{ mm})$ | **$Z = +18\text{ mm}$** (Elevated Platform) |
| **ArUco #3** | Outer Back | $(720\text{ mm}, 540\text{ mm})$ | **$Z = +18\text{ mm}$** (Elevated Platform) |

> [!IMPORTANT]
> **Strict Coplanarity**: All 4 markers are mounted directly on the **elevated platform floor ($Z = +18\text{ mm}$)**. They form an exact **$360\text{ mm} \times 480\text{ mm}$ rectangular planar grid**. Markers must be printed on matte vinyl with zero glare.

---

## 10. Cable Looming & Operational Protocols

1. **Dynamic Torsion Sleeving**: Cables exiting the base of each xArm 1s are wrapped in flexible spiral wrap and anchored to the rear baseplate mounting hole with a strain-relief zip tie loop. This prevents JST connector fatigue over the $200^\circ$ rotation cycle.
2. **Cube Scattering Protocol**:
   - Cubes are manually or semi-automatically scattered across the elevated right-side arena.
   - Initial separation rule: **$\ge 15\text{ mm}$ edge-to-edge separation**.
   - Cubes must not be stacked upon initial placement.

