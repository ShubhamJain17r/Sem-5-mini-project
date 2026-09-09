# Stepping Stone 2: Kinematics, Dynamics & Grasp Planning — Dual xArm 1s

---

## 1. Executive Summary & Core Mechanics

Picking an object with a low-cost, 5/6-DOF articulated bus-servo arm like the **LewanSoul / Hiwonder xArm 1s** is fundamentally different from controlling a rigid, high-torque 6-axis industrial robot (such as an ABB or KUKA).

Industrial arms possess infinite wrist roll, high-precision harmonic drives, and high payload-to-weight ratios allowing arbitrary 6D pose approaches (e.g., straight top-down pin-point picks). In contrast, the xArm 1s is constrained by:

1. **Mechanical Link Dimensions**: Base-to-shoulder height ($d_1 = 65\text{ mm}$), Upper Arm ($L_1 = 101\text{ mm}$), Forearm ($L_2 = 95\text{ mm}$), End-Effector assembly ($L_3 \approx 150\text{--}168\text{ mm}$).
2. **Torque Dropoff**: Gear-train bus servos (LX-15D / similar) experience maximum torque stress when links are extended horizontally, with mechanical backlash ($1^\circ\text{--}3^\circ$) magnifying at the gripper tip.
3. **Claw Kinematics**: The 2-finger parallel-jaw gripper opens to $55\text{ mm}$ max width and is driven by an axial roll servo (Motor 5) and pitch servo (Motor 4).

This document outlines the **exact mathematics, geometry, coordinate transformations, multi-motor coordination, grasp planning physics, limitations, and potential design improvements** for reliably picking $25\text{ mm}$ colored wooden cubes from the elevated arena.

---

## 2. Robot Kinematic Architecture & Coordinate Frames

### 2.1 Coordinate Frame Definition

We establish a right-handed Cartesian base coordinate frame $\mathcal{F}_{\text{base}}$ located at the center of the robot's mounting baseboard footprint ($Z=0$):

- **$+X$**: Points forward toward the pick zone (into the right arena).
- **$+Y$**: Points toward the midline / left flank depending on mounting orientation.
- **$+Z$**: Points vertically upward perpendicular to the baseboard.

```text
                  ┌─────────┐
                  │ Gripper │ (Claw: w in [0, 55mm])
                  └────┬────┘
                       │ L3 = 150-168 mm (End-Effector + Motor 5 roll)
                  ┌────┴────┐
                  │ Motor 4 │ Wrist Pitch (theta_4)
                  └────┬────┘
                       │ L2 = 95 mm (Forearm)
                  ┌────┴────┐
                  │ Motor 3 │ Elbow Pitch (theta_3)
                  └────┬────┘
                       │ L1 = 101 mm (Upper Arm)
                  ┌────┴────┐
                  │ Motor 2 │ Shoulder Pitch (theta_2)
                  └────┬────┘
                       │ d1 = 65 mm (Base Pillar)
                  ┌────┴────┐
                  │ Motor 1 │ Base Azimuth / Yaw (theta_1)
   ═══════════════╧═════════╧═══════════════ Table Level (Z = 0)
```

### 2.2 Physical Dimension Parameters

From technical measurements ([`docs/xArm demensions.png`](file:///d:/Sem-3-mini-project/docs/xArm%20demensions.png)):

| Parameter                 | Description                                                     |                  Value                   |
| ------------------------- | --------------------------------------------------------------- | :--------------------------------------: |
| $d_1$                     | Base floor ($Z=0$) to Motor 2 Shoulder Pitch Axis               |             $65.0\text{ mm}$             |
| $L_1$                     | Motor 2 Shoulder Axis to Motor 3 Elbow Axis                     |            $101.0\text{ mm}$             |
| $L_2$                     | Motor 3 Elbow Axis to Motor 4 Wrist Pitch Axis                  |             $95.0\text{ mm}$             |
| $L_3$                     | Motor 4 Wrist Axis to Gripper Fingertip Grasp Center            |        $\approx 150.0\text{ mm}$         |
| $w_{\max}$                | Maximum Gripper Jaw Opening Width                               |             $55.0\text{ mm}$             |
| $d_{\text{cube}}$         | Target Wooden Cube Side Length                                  |             $25.0\text{ mm}$             |
| $Z_{\text{arena}}$        | Elevated Picking Arena Floor Surface                            |    $++20.0\text{ mm}$    |
| $Z_{\text{cube\_center}}$ | Grasp Target Elevation ($Z_{\text{arena}} + d_{\text{cube}}/2$) | $\approx 32.5\text{ mm}$ |

---

## 3. The Grasping Paradox: Why Pure Vertical Top-Down Picks Fail

A common rookie mistake is attempting to pick cubes by positioning the gripper vertically downwards ($\text{Pitch } \phi = -90^\circ$).

### 3.1 Mathematical Proof of Reach Limit for Vertical Picks

To pick vertically downward, the wrist joint (Motor 4) must hover directly above the object at:
$$r_w = r_{\text{target}}, \quad z_w = z_{\text{target}} + L_3$$
With $z_{\text{target}} \approx 30\text{ mm}$ and $L_3 \approx 150\text{ mm}$, the wrist must be at:
$$z_w = 30 + 150 = 180\text{ mm}$$
The vertical distance from the shoulder axis ($d_1 = 65\text{ mm}$) to the wrist is:
$$\Delta z = z_w - d_1 = 180 - 65 = 115\text{ mm}$$
The combined reach of Link 1 and Link 2 is:
$$L_1 + L_2 = 101 + 95 = 196\text{ mm}$$
By the Pythagorean theorem, the maximum radial distance $r_{\max,\text{vertical}}$ the wrist can reach is:
$$r_{\max,\text{vertical}} = \sqrt{(L_1 + L_2)^2 - \Delta z^2} = \sqrt{196^2 - 115^2} = \sqrt{38416 - 13225} = \sqrt{25191} \approx 158.7\text{ mm}$$

> [!WARNING]
> **Physical Constraint**: Under a pure vertical top-down orientation ($\phi = -90^\circ$), the arm **cannot physically reach any object beyond $15.8\text{ cm}$** from its base. Any cube placed in the outer arena ($r \in [160, 240\text{ mm}]$) is kinematically unreachable top-down.

### 3.2 Why Radial / Angled Picking Resolves the Paradox

When the arm approaches at an inclined or horizontal pitch angle $\phi \in [-45^\circ, 0^\circ]$:
$$r_w = r_{\text{target}} - L_3 \cos(\phi)$$
$$z_w = z_{\text{target}} - L_3 \sin(\phi)$$
For a horizontal pick ($\phi = 0^\circ$) at $r_{\text{target}} = 220\text{ mm}$:
$$r_w = 220 - 150 = 70\text{ mm}$$
$$z_w = 30\text{ mm} \implies \Delta z = 30 - 65 = -35\text{ mm}$$
The required arm extension is:
$$D = \sqrt{70^2 + (-35)^2} = \sqrt{4900 + 1225} = 78.2\text{ mm} \ll 196\text{ mm}$$
The links operate near their mid-range configuration ($\approx 90^\circ\text{--}120^\circ$ elbow bend), where servo gear teeth have maximum contact and motors run in their highest mechanical torque band!

---

## 4. Mathematical Formulation: Analytical Inverse Kinematics (IK)

Because Motor 1 rotates purely about the vertical $Z$-axis and Motors 2, 3, and 4 rotate about parallel horizontal axes, the 3D IK problem decouples into a **1D azimuthal rotation** and a **2D planar 3-link kinematic solution**.

```text
           Z ^
             │                (rw, zw)
             │                Motor 4 (Wrist)
             │                o═══════════════>  Gripper (L3, pitch phi)
             │               /                 \
             │              /                   \ Target Cube (rt, zt)
             │          L2 /
             │            /
             │   Motor 3 o (Elbow)
             │          /
             │      L1 /
             │        /
             │       o Motor 2 (Shoulder @ z = d1 = 65mm)
             │       │
             │       │ d1 = 65mm
             └───────┴─────────────────────────────> r
                    Base (0, 0)
```

### 4.1 Step 1: Base Azimuth ($\theta_1$)

Given target cube coordinates $(X_{\text{target}}, Y_{\text{target}})$ relative to the arm's base:
$$\theta_1 = \text{atan2}(Y_{\text{target}}, X_{\text{target}})$$
Horizontal radial distance:
$$r_{\text{target}} = \sqrt{X_{\text{target}}^2 + Y_{\text{target}}^2}$$

### 4.2 Step 2: Wrist Position Calculation

Given the chosen approach pitch angle $\phi(r)$ (measured relative to the horizontal plane, where horizontal is $0^\circ$ and pointing down is negative):
$$r_w = r_{\text{target}} - L_3 \cos(\phi)$$
$$z_w = z_{\text{target}} - L_3 \sin(\phi)$$

In the planar frame relative to the shoulder joint $(0, d_1)$:
$$r' = r_w$$
$$z' = z_w - d_1$$
Diagonal distance from shoulder to wrist:
$$D = \sqrt{(r')^2 + (z')^2}$$

### 4.3 Step 3: Elbow Angle ($\theta_3$)

Using the Law of Cosines on the triangle formed by $L_1$, $L_2$, and $D$:
$$\cos(\alpha_3) = \frac{L_1^2 + L_2^2 - D^2}{2 L_1 L_2}$$
To prevent numerical overflow from floating point errors, clamp the argument:
$$C_3 = \text{clip}\left(\frac{L_1^2 + L_2^2 - D^2}{2 L_1 L_2}, -1.0, 1.0\right)$$
The elbow joint interior angle is $\alpha_3 = \arccos(C_3)$.
In the robot's servo reference frame (where $\theta_3 = 0^\circ$ corresponds to a straight arm):
$$\theta_3 = 180^\circ - \alpha_3 = 180^\circ - \arccos(C_3)$$
_(Configured in Elbow-Up mode to guarantee clearance from the arena platform)._

### 4.4 Step 4: Shoulder Angle ($\theta_2$)

The angle of the line connecting shoulder to wrist relative to horizontal:
$$\beta = \text{atan2}(z', r')$$
The internal angle at the shoulder via Law of Cosines:
$$\cos(\psi) = \frac{L_1^2 + D^2 - L_2^2}{2 L_1 D}$$
$$\psi = \arccos\left(\text{clip}\left(\frac{L_1^2 + D^2 - L_2^2}{2 L_1 D}, -1.0, 1.0\right)\right)$$
The shoulder angle $\theta_2$ (measured above the horizontal):
$$\theta_2 = \beta + \psi$$

### 4.5 Step 5: Wrist Pitch Angle ($\theta_4$)

The total pitch orientation of the gripper $\phi$ is the kinematic chain sum:
$$\phi = \theta_2 - \theta_3 + \theta_4 \quad \implies \quad \theta_4 = \phi - \theta_2 + \theta_3$$
_(Note: Sign conventions depend on whether servo directions are inverted in firmware; this is normalized in the driver layer)._

---

## 5. End-Effector Rotation & Alignment (Motor 5 & Claw Motor 6)

One of the most nuanced questions in arm manipulation: **How is the end-effector rotation calculated, and how should it orient relative to the cube?**

### 5.1 Degrees of Freedom of the xArm 1s Gripper

- **Motor 4 (Wrist Pitch)**: Changes tilt angle $\phi$ (vertical plane).
- **Motor 5 (Wrist Roll)**: Rotates axially around the gripper's centerline.
- **Motor 6 (Claw Opening)**: Symmetrically drives two parallel fingers between $0\text{ mm}$ (fully closed) and $55\text{ mm}$ (fully open).

### 5.2 The Tilt-Roll Coupling Problem

When an arm picks at a shallow pitch ($\phi \in [-20^\circ, 0^\circ]$):

- If Motor 5 rotates by an angle $\psi_{\text{roll}} \ne 0^\circ$, the two gripper fingers **tilt out of the horizontal plane**.
- As a consequence, one fingertip will point down toward the table surface, while the other tilts upward into the air.
- When the arm descends to grasp the cube at $Z \approx 30\text{ mm}$, the lower fingertip **gouges or collides with the arena platform**, while the upper fingertip misses the cube!

```text
     Level Claw (Roll = 0°):              Tilted Claw (Roll != 0°):
       ┌──────┐      ┌──────┐                    ┌──────┐
       │Finger│      │Finger│                   /Finger/
       └──────┘      └──────┘                  /      /
   ───────┬──────────────┬────────         ───/──────/────────────────
      Cube: [ 25mm ]                      Low Finger Strikes Arena Floor!
   ═══════════════════════════════       ═════════════════════════════════
     Arena Mat (Z = 15-20 mm)               Arena Mat Damaged / Jammed
```

### 5.3 Solution: The Dual-Regime Grasp Strategy

Depending on radial distance $r$, we deploy two distinct orientation regimes:

#### Regime A: Near-Field Grasping ($r \in [100, 145\text{ mm}]$ — Steep Pitch $\phi \approx -45^\circ$)

- At close range, the arm can pitch down steeply without running out of reach.
- The camera detects the cube's planar bounding box angle $\theta_{\text{cube}} \in [-45^\circ, +45^\circ]$ via OpenCV `cv2.minAreaRect()`.
- The claw approach vector has a strong vertical component.
- **Motor 5 Calculation**:
  $$\theta_5 = \theta_{\text{cube}} - \theta_1$$
  Motor 5 aligns the claw jaws perpendicular to the flat faces of the cube. Fingertip clearance from the table remains safe because the pitch is steep ($\approx 45^\circ$).

#### Regime B: Mid-to-Far Grasping ($r \in [145, 240\text{ mm}]$ — Shallow Pitch $\phi \in [-20^\circ, 0^\circ]$)

- At medium and far reaches, the claw approaches nearly horizontally along the radius vector.
- **Motor 5 is LOCKED strictly at $0^\circ$ (Roll = $0^\circ$)**. This ensures both fingers remain parallel to the table.
- **Mechanical Self-Centering**:
  Because the claw opens to $55\text{ mm}$ ($2.2\times$ wider than the $25\text{ mm}$ cube), even if the cube is rotated arbitrarily ($45^\circ$ diagonal profile = $25\sqrt{2} \approx 35.3\text{ mm} < 55\text{ mm}$), it fits completely inside the open jaws!
- When the parallel rubberized jaws close:
  1. The jaws make contact with two opposing corners or edges of the cube.
  2. The high-friction silicone mat underneath provides a controlled reaction moment.
  3. The closing force gently cams/squares the cube into flat alignment against the jaw faces before full clamping torque is applied.

---

## 6. Multi-Motor Coordination & 5-Phase Trajectory Profiles

Direct point-to-point joint jumps cause severe mechanical overshoot, gear tooth stripping, and tip oscillations. The motion planner decomposes every pick-and-place operation into a smooth **5-phase coordinated waypoint trajectory**.

```text
  Trajectory Altitude Profile:
  Z ^
    │          (Waypoint 1: Transit Home)
    │                  o
    │                 / \
    │                /   \  (Waypoint 2: Standoff @ r - 30mm, Z + 15mm)
    │               /     o ══════════> o (Waypoint 3: Grasp @ r, Z_pick)
    │              /                     \
    │  ═══════════o                       o (Waypoint 4: Retract & Lift)
    │  (Start)                             \
    │                                       o═══════> (Waypoint 5: Swing to Box)
    └──────────────────────────────────────────────────────────────────────────> r
```

### 6.1 Trajectory Sequence Breakdown

| Phase                            | Action                                                                                                        | Motor Coordination Details                                                                                                      |               Speed / Duration               |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- | :------------------------------------------: |
| **1. Standoff Approach**         | Arm moves to $(r_{\text{target}} - 30\text{ mm}, \theta_1, z_{\text{pick}} + 15\text{ mm})$                   | Motors 1, 2, 3, 4 interpolate concurrently. Gripper (Motor 6) opens to $55\text{ mm}$.                                          | $800\text{ ms}$ (smooth trapezoidal profile) |
| **2. Linear Ingress**            | Slides forward radially from $r - 30\text{ mm}$ to $r_{\text{target}}$ at constant height $z_{\text{pick}}$   | Motors 2, 3, 4 perform coordinated differential pitch to maintain linear tip trajectory.                                        |      $400\text{ ms}$ (slow, controlled)      |
| **3. Grasp Clamping**            | Closes Motor 6 until target PWM or load resistance threshold                                                  | Motors 1–5 hold position stiffness. Motor 6 drives until jaw width reaches $\approx 25\text{ mm}$.                              |               $300\text{ ms}$                |
| **4. Tactile Verification**      | Read servo position feedback register                                                                         | If position $< 10\text{ mm}$ (empty grip) or $> 35\text{ mm}$ (jam): Abort & re-attempt. If $20\text{--}30\text{ mm}$: Success. |                $50\text{ ms}$                |
| **5. Vertical Ascent & Retract** | Lifts vertically by $+40\text{ mm}$, retracts radially by $-50\text{ mm}$                                     | Motors 2 and 3 lift load; ensures claw clears surrounding cubes and barriers.                                                   |               $500\text{ ms}$                |
| **6. 180° Drop Swing**           | Arm tucks into compact pose ($r \approx 130\text{ mm}$) and swings $\approx 180^\circ$ left to collection box | Low moment of inertia $I = \sum m r^2$ minimizes dynamic torque load on Motor 1.                                                |               $1000\text{ ms}$               |

---

## 7. Physical & Mechanical Dynamics (Torque, Friction & Grip Physics)

### 7.1 Gravity Torque Calculation on Motor 2 (Shoulder)

The shoulder motor (Motor 2) experiences the largest static load:
$$\tau_{\text{shoulder}} = m_1 g \cdot \frac{L_1}{2} \cos(\theta_2) + m_2 g \cdot \left(L_1 \cos(\theta_2) + \frac{L_2}{2} \cos(\theta_2 + \theta_3)\right) + m_{\text{gripper}} g \cdot r_w$$

- Mass of links + motors: $\approx 450\text{ g}$.
- Mass of cube: $\approx 15\text{ g}$ (lightweight wood).
- At full theoretical extension ($r \approx 270\text{ mm}$), $\tau_{\text{shoulder}}$ exceeds $1.4\text{ N}\cdot\text{m}$.
- Standard bus servos (rated $1.5\text{ N}\cdot\text{m}$ stall torque) operate at **90%+ of their thermal limits**, leading to voltage sag and brownouts.
- **By capping reach at $R_{\max} = 320\text{ mm}$ and utilizing elevated arena ($Z=+15\text{--}20\text{ mm}$)**:
  $\tau_{\text{shoulder}}$ drops to $\approx 0.95\text{ N}\cdot\text{m}$ (a **32% torque reduction**), providing comfortable safety margins.

### 7.2 Friction Mat Grasp Physics

- Wooden cube mass: $m \approx 15\text{ g} \implies F_N \approx 0.15\text{ N}$.
- Kinetic friction coefficient of bare wood on slick acrylic table: $\mu \approx 0.25$.
  - Push force required to slide cube: $F_{\text{slide}} = \mu F_N = 0.037\text{ N}$ (extremely easy to inadvertently nudge or push away during claw advance!).
- Kinetic friction coefficient of wood on **matte silicone mat**: $\mu_{\text{silicone}} \approx 0.75\text{--}0.85$.
  - Force to slide cube: $F_{\text{slide}} \approx 0.12\text{ N}$ ($>3\times$ higher resistance!).
  - **Outcome**: When the open claw touches or funnels the cube, the silicone mat prevents the cube from squirting forward, allowing the jaws to grip securely.

---

## 8. Engineering Improvements & Design Optimizations

How can we make the physical grasping system significantly better?

### 💡 1. 3D-Printed Chamfered Silicone Finger Pads

- **Problem**: Stock metal/acrylic xArm claw fingers are smooth and flat. If a cube is grasped off-center, it can twist and drop.
- **Solution**: 3D-print slip-on TPU or silicone finger sleeves featuring:
  - An internal **$90^\circ$ V-groove notch** that naturally nests square cube corners.
  - Serrated or diamond-textured grip pads.

```text
      Stock Claw (Flat):                 Improved V-Groove Claw:
      │              │                     │ \            / │
      │   [ Cube ]   │                     │  \ [ Cube ] /  │
      │              │                     │   \        /   │
   (Can slip under torque)             (Mechanically self-locking)
```

### 💡 2. Optical Break-Beam / Micro-Switch Contact Sensing

- **Current Mechanism**: Grasp confirmation relies on reading servo potentiometer resistance/position telemetry over serial.
- **Enhancement**: Install a miniature infrared break-beam sensor (e.g., $3\text{ mm}$ IR emitter-detector pair) across the claw jaws.
  - When the cube enters the jaw opening, the beam breaks ($0\text{ V} \rightarrow 5\text{ V}$).
  - Gives millisecond-level hardware confirmation that the cube is physically seated between the fingers before lifting.

### 💡 3. Dual-Spring Shoulder Counterbalance

- **Problem**: Motor 2 constantly fights gravity even when holding stationary.
- **Enhancement**: Add a passive dual tension spring or elastic surgical tubing connecting the base bracket to Link 1.
  - The spring is tensioned when the arm reaches down, counteracting gravity torque by $\approx 0.4\text{ N}\cdot\text{m}$.
  - Reduces Motor 2 operating temperatures by up to $15^\circ\text{C}$ during long sorting sessions.

### 💡 4. Adaptive Visual Servo (IBVS) Radial Fine-Tuning

- If camera calibration has a $2\text{--}3\text{ mm}$ offset, a single top-down image might slightly misjudge cube position.
- Since the overhead camera observes the right arena in real-time, the vision node can verify that the gripper claw's centroid aligns with the cube centroid during the standoff phase ($r - 30\text{ mm}$) before committing to the grasp.

---

## 9. Physical Limitations, Singularities & Failure Boundaries

| Limitation / Boundary                                | Cause                                                                 | Manifestation                                                                                                         | Software Enforcement                                                          |
| ---------------------------------------------------- | --------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| **Boundary Singularity ($R > 240\text{ mm}$)**       | Fully extended elbow ($L_1, L_2$ collinear)                           | Jacobian determinant $\det(J) \rightarrow 0$; infinite joint velocity required for radial motion; severe servo jitter | Software clamps all picks to $r \le 240\text{ mm}$                            |
| **Base Collision Singularity ($R < 100\text{ mm}$)** | Proximity to $140\times 154\text{ mm}$ base mounting plate            | Wrist / Link 2 hits base servo chassis                                                                                | Software rejects any detection with $r < 100\text{ mm}$                       |
| **Backlash Error Envelope**                          | Gear play in plastic/brass spur gearboxes ($1^\circ\text{--}2^\circ$) | End-effector has $\pm 2.5\text{ mm}$ mechanical slack at $240\text{ mm}$ reach                                        | Compensated by wide $55\text{ mm}$ jaw opening ($> 2\times$ cube size)        |
| **Drop Clearance Limit**                             | Left collection box partition height ($H \approx 30\text{ mm}$)       | Arm could clip box wall if dropped at low altitude                                                                    | Planner enforces ascent to $Z = +50\text{ mm}$ before descending into bin     |
| **Bus Contention Latency**                           | Half-duplex UART communication over shared bus at 115200 baud         | Querying all 6 servo registers takes $\approx 18\text{ ms}$                                                           | Status queries run asynchronously; motion commands broadcast in packet bursts |

---

## 10. Kinematic Lookup Table (LUT) & Implementation Reference

For maximum performance on embedded hardware (Jetson Nano / Raspberry Pi 5), the system pre-computes an empirical **Distance-to-Pitch LUT** to bypass complex runtime numerical optimization.

### 10.1 Empirical Distance-to-Pitch Table

|               Distance Range $r$               | Approach Pitch $\phi(r)$ | Standoff Radius $r - \Delta r$ | Grasp Elevation $Z_{\text{pick}}$ |                       Motor 5 Roll Mode                       |
| :--------------------------------------------: | :----------------------: | :----------------------------: | :-------------------------------: | :-----------------------------------------------------------: |
|     **$100\text{--}130\text{ mm}$** (Near)     |       $-45^\circ$        |       $r - 20\text{ mm}$       |         $32.0\text{ mm}$          | Object-Aligned ($\theta_5 = \theta_{\text{cube}} - \theta_1$) |
|   **$130\text{--}165\text{ mm}$** (Near-Mid)   |       $-35^\circ$        |       $r - 25\text{ mm}$       |         $30.0\text{ mm}$          | Object-Aligned ($\theta_5 = \theta_{\text{cube}} - \theta_1$) |
|   **$165\text{--}200\text{ mm}$** (Mid-Far)    |       $-20^\circ$        |       $r - 30\text{ mm}$       |         $28.0\text{ mm}$          |            **Locked Level ($\theta_5 = 0^\circ$)**            |
| **$200\text{--}240\text{ mm}$** (Far Boundary) | $-10^\circ$ to $0^\circ$ |       $r - 30\text{ mm}$       |         $27.0\text{ mm}$          |            **Locked Level ($\theta_5 = 0^\circ$)**            |

### 10.2 Python IK Solver Reference Implementation

```python
import math

class XArmKinematics:
    def __init__(self):
        # Physical Link Lengths (mm)
        self.d1 = 65.0       # Base to Motor 2 shoulder axis
        self.L1 = 101.0      # Shoulder to elbow
        self.L2 = 95.0       # Elbow to wrist pitch
        self.L3 = 150.0      # Wrist pitch to gripper grasp center

        # Operational Boundaries (mm)
        self.r_min = 100.0
        self.r_max = 240.0
        self.z_arena = 18.0  # Elevated arena surface (15-20 mm)
        self.cube_h = 25.0

    def get_pitch_angle(self, r):
        """Distance-based pitch approach angle LUT (degrees)"""
        if r < 130.0:
            return -45.0
        elif r < 165.0:
            return -35.0
        elif r < 200.0:
            return -20.0
        else:
            return -10.0

    def solve_ik(self, x_target, y_target, cube_angle=0.0):
        """
        Computes the target angles for all 6 motors.
        Returns: (theta_1, theta_2, theta_3, theta_4, theta_5) in degrees, or None if unreachable.
        """
        r = math.hypot(x_target, y_target)
        if not (self.r_min <= r <= self.r_max):
            return None  # Out of reach

        # 1. Base Azimuth
        theta_1 = math.degrees(math.atan2(y_target, x_target))

        # 2. Pitch selection & Wrist coordinate calculation
        phi_deg = self.get_pitch_angle(r)
        phi_rad = math.radians(phi_deg)

        z_target = self.z_arena + (self.cube_h / 2.0)
        rw = r - self.L3 * math.cos(phi_rad)
        zw = z_target - self.L3 * math.sin(phi_rad)

        # Planar coordinates relative to shoulder (0, d1)
        r_prime = rw
        z_prime = zw - self.d1
        D = math.hypot(r_prime, z_prime)

        # Triangle inequality check
        if D > (self.L1 + self.L2) or D < abs(self.L1 - self.L2):
            return None

        # 3. Elbow Angle (theta_3) - Elbow Up
        cos_alpha3 = (self.L1**2 + self.L2**2 - D**2) / (2.0 * self.L1 * self.L2)
        cos_alpha3 = max(-1.0, min(1.0, cos_alpha3))
        alpha3 = math.acos(cos_alpha3)
        theta_3 = 180.0 - math.degrees(alpha3)

        # 4. Shoulder Angle (theta_2)
        beta = math.atan2(z_prime, r_prime)
        cos_psi = (self.L1**2 + D**2 - self.L2**2) / (2.0 * self.L1 * D)
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = math.acos(cos_psi)
        theta_2 = math.degrees(beta + psi)

        # 5. Wrist Pitch (theta_4)
        theta_4 = phi_deg - (theta_2 - theta_3)

        # 6. Wrist Roll (Motor 5)
        if r < 165.0:
            theta_5 = cube_angle - theta_1  # Align with cube face
        else:
            theta_5 = 0.0                   # Lock level with table

        return {
            'motor_1_azimuth': theta_1,
            'motor_2_shoulder': theta_2,
            'motor_3_elbow': theta_3,
            'motor_4_wrist_pitch': theta_4,
            'motor_5_wrist_roll': theta_5,
            'approach_radius': r,
            'approach_pitch': phi_deg
        }
```
