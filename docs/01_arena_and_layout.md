# Stepping Stone 1: Arena & Physical Component Placement

This document details the complete mechanical construction, spatial dimensions, component placements, physical barriers, lighting, wiring, and electrical layout for the dual xArm 1s color-sorting robotic workcell.

---

## 1. Workcell Dimensions & Global Coordinate System

The entire workcell is constructed on a single rigid wooden or aluminum baseboard measuring **$800\text{ mm} \text{ (Width)} \times 600\text{ mm} \text{ (Depth)}$**.

```text
  Y (Depth: 600 mm)
   ^
   │  ┌────────────────────────────────────────────────────────────┐
   │  │ BACK OF WORKSPACE (600 mm)                                 │
   │  │                                                            │
   │  │   [Box 1 - NW]          xArm 1 (Back @ 420mm)              │
   │  │                                        \                   │
   │  │                                         \ EXPANDED         │
   │  │   [Central Electronics   Collinear Axis  │ STADIUM ARENA   │
   │  │    Bay: Jetson Orin & Hub]    (X = 360 mm)    │ (Elevated 20mm) │
   │  │                                         /  (> 2,100 cm²)   │
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

| Manipulator            |    Center Coordinate $(X, Y)$    |        Mounting Elevation         | Operating Role                                                                 |
| ---------------------- | :------------------------------: | :-------------------------------: | ------------------------------------------------------------------------------ |
| **xArm 2 (Front Arm)** | $(360\text{ mm}, 180\text{ mm})$ | $Z = 0\text{ mm}$ (Flat on Table) | Covers front arena sector ($Y < 300\text{ mm}$) & drops into South-West Box 2  |
| **xArm 1 (Back Arm)**  | $(360\text{ mm}, 420\text{ mm})$ | $Z = 0\text{ mm}$ (Flat on Table) | Covers back arena sector ($Y \ge 300\text{ mm}$) & drops into North-West Box 1 |

### 2.1 Kinematic Spacing Verification

- **Center-to-Center Spacing**: $S = 420 - 180 = \mathbf{240\text{ mm}}$.
- **Baseplate Dimensions**: $140\text{ mm} \text{ (Width along } X) \times 154\text{ mm} \text{ (Length along } Y)$ ([`docs/xArm demensions.png`](xArm%20demensions.png)).
- **Baseplate Center-to-Edge Offset**:
  $$d_{\text{offset}} = \frac{140\text{ mm}}{2} = \mathbf{70\text{ mm}}$$
  _(From the Motor 1 central yaw rotation axis to the outer flat edge facing the picking arena)_.
- **Baseplate Air Gap**: With baseplate length of $154\text{ mm}$ along $Y$:
  $$\text{Gap}_{\text{bases}} = 240 - 154 = \mathbf{86\text{ mm}}$$
- **Table Margins**: Front and back margins are symmetrically identical at $180\text{ mm}$ ($103\text{ mm}$ clear from baseplate edge to table edge), leaving ample space for ArUco markers and boom clamping.
- **Midline Reach**: Midline partition is at $Y = 300\text{ mm}$ ($120\text{ mm}$ from each arm center). Both arms easily cover the midline without strain.

---

## 3. Picking Arena Dimensions & Dual Coordinate Systems

### 3.1 Resolving the Edge vs. Center Coordinate Measurement

- **User Reference ($d_{\text{edge}}$)**: Distance measured from the **outer flat edge of the baseplate** (suction cup plate) into the arena.
- **Kinematic Center Reference ($R_{\text{center}}$)**: Distance measured from the **central axis of rotation of Motor 1** used in Forward/Inverse Kinematics and polar transformations.
- **Exact Conversion**:
  $$R_{\text{center}} = d_{\text{edge}} + 70\text{ mm}$$

### 3.2 Operating Boundaries

| Boundary                        | From Baseplate Edge ($d_{\text{edge}}$) | From Robot Center ($R_{\text{center}}$) | Engineering Rationale                                                                                                                                         |
| ------------------------------- | :-------------------------------------: | :-------------------------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Inner Boundary ($R_{\min}$)** |   **$50\text{ mm}$ ($5\text{ cm}$)**    |  **$120\text{ mm}$ ($12\text{ cm}$)**   | Provides $50\text{ mm}$ clearance from baseplate edge; prevents claw collision with base screws and avoids camera blindspots.                                 |
| **Outer Boundary ($R_{\max}$)** |  **$250\text{ mm}$ ($25\text{ cm}$)**   |  **$320\text{ mm}$ ($32\text{ cm}$)**   | Leaves $24\text{ mm}$ safety margin before absolute straight-arm lock ($274\text{ mm}$ from edge); keeps elbow at $\approx 122^\circ$ to prevent motor stall. |
| **Absolute Mechanical Limit**   |   $274\text{ mm}$ ($27.4\text{ cm}$)    |   $344\text{ mm}$ ($34.4\text{ cm}$)    | Straight-arm reach ($L_1 + L_2 + L_3$); elbow locked at $180^\circ$ singularity.                                                                              |

- **Radial Depth**: $\Delta R = 320 - 120 = \mathbf{200\text{ mm}} = \mathbf{20\text{ cm}}$ (a $+43\%$ increase over earlier $14\text{ cm}$ span).
- **Usable Arena Surface Area**: $> \mathbf{2{,}100\text{ cm}^2}$ (spans $\approx 370\text{ mm} \times 500\text{ mm}$, comfortably fitting 25–30 scattered $25\text{ mm}$ cubes).
- **Right Baseboard Margin**: Outer edge of arena reaches $X = 360 + 320 = \mathbf{680\text{ mm}}$, leaving a generous $120\text{ mm}$ ($12\text{ cm}$) margin on the $800\text{ mm}$ board.

### 3.3 Elevation Setup (+20 mm)

- The picking arena platform is elevated by **$+20\text{ mm}$** above the baseboard ($Z = +20\text{ mm}$).
- Lined with a high-friction matte silicone mat ($\mu \approx 0.8$).
- Cube center sits at $Z_{\text{cube}} = 20 + 12.5 = \mathbf{32.5\text{ mm}}$.
- Motor 2 shoulder axis sits at $Z = 65\text{ mm}$.
- **Result**: The vertical drop from Motor 2 to cube center is only $65 - 32.5 = \mathbf{32.5\text{ mm}}$, enabling a near-horizontal grasp approach ($\phi \approx -5^\circ$) at $d_{\text{edge}} = 25\text{ cm}$ without requiring any complex grooves or baseboard cutting.

---

## 4. Drop Zone Architecture: Solving the $240^\circ$ Base Servo ROM

### 4.1 The Hardware Constraint

The **LewanSoul / Hiwonder LX-15D** bus servos driving the arm base yaw have a maximum mechanical travel range of **$240^\circ$** ($0\text{ to }1000$ tick range).

- If the servo's mechanical center ($120^\circ$) points East toward the picking arena ($0^\circ$), it can only swing:
  - Counter-Clockwise (CCW): up to $+120^\circ$ (North-West)
  - Clockwise (CW): down to $-120^\circ$ (South-West)
- **Due West ($180^\circ$, directly behind the arm) is a physical deadzone.** Attempting a $180^\circ$ drop swing causes servo stall or internal gear damage.

### 4.2 The Angled NW / SW Drop Box Solution

By moving the collection boxes to the **North-West (NW)** and **South-West (SW)** corners of each arm's reachable arc, the drop motion remains well within the $240^\circ$ physical limit:

```text
  North-West Drop:                  South-West Drop:
  (Arm 1 @ Y = 420 mm)              (Arm 2 @ Y = 180 mm)
         ^                                 |
         │ \ Angle: +125°                  │ \ Angle: -125°
         │  \                              │  \
         │   ▼                             │   ▼
     [Box 1 (NW)]                      [Box 2 (SW)]
  Total Travel: 205° <= 240°        Total Travel: 205° <= 240°
```

| Component                      |    Coordinate Center $(X, Y)$    |     Elevation     |       Angular Orientation from Arm Base       |
| ------------------------------ | :------------------------------: | :---------------: | :-------------------------------------------: |
| **Box 1 (Dedicated to Arm 1)** | $(115\text{ mm}, 500\text{ mm})$ | $Z = 0\text{ mm}$ | **North-West** ($\approx +125^\circ$ azimuth) |
| **Box 2 (Dedicated to Arm 2)** | $(115\text{ mm}, 100\text{ mm})$ | $Z = 0\text{ mm}$ | **South-West** ($\approx -125^\circ$ azimuth) |
| **Central Electronics Bay**    | $(115\text{ mm}, 300\text{ mm})$ | $Z = 0\text{ mm}$ |   Centered between boxes on the left flank    |

- **Arm 1 Travel Envelope**: Picks between $-23^\circ$ (midline) and $+24^\circ$ (back-right). Swings CCW to $+125^\circ$ for drop. Total span: $\approx 148^\circ \le 205^\circ < 240^\circ$.
- **Arm 2 Travel Envelope**: Picks between $-24^\circ$ (front-right) and $+23^\circ$ (midline). Swings CW to $-125^\circ$ for drop. Total span: $\approx 148^\circ \le 205^\circ < 240^\circ$.

---

## 5. Central Electronics & Power Bay

Relocating Box 1 to the North-West and Box 2 to the South-West opens up a large, unutilized rectangular area on the left flank ($X \in [55, 175\text{ mm}], Y \in [215, 385\text{ mm}]$).

This area is designated as the **Central Electronics & Wiring Bay**:

```text
┌─────────────────────────────────────────────────────────────┐
│                 CENTRAL ELECTRONICS BAY                     │
│               (X: 55-175 mm, Y: 215-385 mm)                 │
├─────────────────────────────────────────────────────────────┤
│  1. NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (Top Sub-Bay)              │
│     - Mount: M2.5 nylon standoffs on baseboard              │
│     - USB runs to Arm 1 & Arm 2: < 25 cm direct serial runs  │
│     - Zero cable drag or snagging during 205° drop swing    │
├─────────────────────────────────────────────────────────────┤
│  2. Dual 7.4V / 10A Power Distribution Hub (Bottom Sub-Bay)  │
│     - 2x Independent Buck Regulators (7.4V @ 5A each)        │
│     - Common ground star-point connection                   │
│     - 1000 µF low-ESR buffer capacitors per arm rail         │
└─────────────────────────────────────────────────────────────┘
```

---

## 6. Optical & Camera Mounting Specifications

1. **Overhead Camera Position**:
   - Centered over the right arena at $(X = 550\text{ mm}, Y = 300\text{ mm})$.
   - Mounted on a rigid boom stand at a height of **$65\text{--}75\text{ cm}$** above the baseboard ($63\text{--}73\text{ cm}$ above the elevated arena surface).
   - Field of View (FOV): Comfortably encloses the $370\text{ mm} \times 480\text{ mm}$ ArUco marker rectangle.
2. **ArUco Marker Placement (Coplanar at $Z = +20\text{ mm}$)**:
   - **ArUco #0**: $(360\text{ mm}, 55\text{ mm})$ — Collinear with Arm 2, Inner Front.
   - **ArUco #1**: $(730\text{ mm}, 55\text{ mm})$ — Outer Front.
   - **ArUco #2**: $(360\text{ mm}, 545\text{ mm})$ — Collinear with Arm 1, Inner Back.
   - **ArUco #3**: $(730\text{ mm}, 545\text{ mm})$ — Outer Back.
   - **Strict Coplanarity**: All 4 markers are mounted directly on the $+20\text{ mm}$ elevated platform to ensure zero homography perspective distortion.
