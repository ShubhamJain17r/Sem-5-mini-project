# Single xArm 1S Dual-Color (Red & Green) Sorting Robot — Implementation Plan

## System Overview

A single LewanSoul / Hiwonder xArm 1S robotic arm autonomously sorts Red and Green wooden craft blocks from a physical **$7 \times 12$ hand-drawn grid arena ($350\text{ mm} \times 600\text{ mm}$, $Z = 0.0\text{ mm}$)** into **symmetric Rear Flank container boxes** (Red box @ Rear-Left, yaw 650 ticks; Green box @ Rear-Right, yaw 354 ticks) using an **overhead rear-flip transfer trajectory**.

The operator views the setup from the **FRONT** (Row 11, $Y = 0.0\text{ mm}$). The entire workcell is constructed on a single **flat wooden platform** ($350\text{ mm} \text{ wide} \times 600\text{ mm} \text{ deep}$). The arm is mounted at the exact center of its base length and width in Rows 3..5, Columns 2..4 at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm}, Z = 0.0\text{ mm})$ with nominal heading **$-90.0^\circ$** (pointing straight South down Column 3 midline):

- **Picking Arena ($7 \times 12$ Grid Geometry)**: Hand-drawn $50.0\text{ mm} \times 50.0\text{ mm}$ square cells across 7 columns (Cols 0..6) and 12 rows (Rows 0..11). Primary autonomous sorting operates in the **Golden Pickup Zone (Rows 7..9, Cols 2..4, $X \in [75, 275\text{ mm}]$, $Y \in [100, 250\text{ mm}]$, $r \in [140.0, 260.0\text{ mm}]$)**. Compliant foam tape ($8\text{--}10\text{ mm}$ tall) retains blocks within the arena.
- **Drop Zones (Dual Rear Flank Container Boxes)**:
  - **Red Container Box**: Rows 0..1, Cols 0..1 at $(X = 50.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, base yaw tick **$650$** ($+35.5^\circ$).
  - **Green Container Box**: Rows 0..1, Cols 5..6 at $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 45.0\text{ mm})$, base yaw tick **$354$** ($-35.5^\circ$).
- **Color Discrimination Methodology**:
  - **Target (Red)**: Dual-band HSV ($[0..10]$ and $[170..180]$) detects red cubes, routed to the Red container box.
  - **Target (Green)**: HSV band ($[38..85]$) detects green cubes, routed to the Green container box.
  - **Chassis Exclusion (Blue)**: Bypassed to eliminate reflections from the blue anodized aluminum robot body.
  - **Spatial Exclusion Mask**: Rectangular mask in image space for Rows 3..5, Cols 2..4 ($X \in [100, 250]$, $Y \in [300, 450]$) zeroes out the robot base.
- **Table-Coplanar ArUco Fiducials**: 4 ArUco markers (`DICT_4X4_50`, IDs 0..3, size $50\times 50\text{ mm}$) mounted flat at $Z = 0.0\text{ mm}$:
  - ID 0: $(25.0, 425.0\text{ mm})$ [Row 3, Col 0] $\to (0, 0)$ Top-Left
  - ID 1: $(325.0, 425.0\text{ mm})$ [Row 3, Col 6] $\to (600, 0)$ Top-Right
  - ID 2: $(25.0, 25.0\text{ mm})$ [Row 11, Col 0] $\to (0, 800)$ Bottom-Left
  - ID 3: $(325.0, 25.0\text{ mm})$ [Row 11, Col 6] $\to (600, 800)$ Bottom-Right
  - Rectified metric grid: $600 \times 800\text{ px}$ at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$).

---

## Hardware Specifications

| Component        | Spec                                                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arm              | 1× LewanSoul / Hiwonder xArm 1S mounted at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$, heading $-90.0^\circ$ (South)                            |
| Baseboard        | Platform board ($350\text{ mm} \times 600\text{ mm} \times 18\text{ mm}$) with hand-drawn $7\times 12$ grid ($5\text{ cm}$ cells)                   |
| Compute          | NVIDIA Jetson Orin Nano / Raspberry Pi 5 / Laptop (Rear Bay $Y \in [500, 600\text{ mm}]$)                                                          |
| Camera           | Logitech C270 or C920 (USB) on scrap aluminum stand @ $H \approx 620\text{ mm}$ nadir over $(175, 250\text{ mm})$                                  |
| Picking Arena    | Golden Pickup Zone (Rows 7..9, Cols 2..4, $Z = 0.0\text{ mm}$, $r \in [140, 260\text{ mm}]$) with compliant foam tape perimeter                     |
| ArUco Setup      | 4 coplanar markers (IDs 0–3, $50\times 50\text{ mm}$) at $(25, 425), (325, 425), (25, 25), (325, 25)$                                               |
| Container Boxes  | Dual physical container boxes ($100\times 100\times 35\text{ mm}$ each): Red at $(50, 550)$, Green at $(300, 550)$                                 |
| Objects          | Wooden craft blocks ($25.4\text{ mm}$ cubes / $60\times 35\text{ mm}$ rectangles): Red and Green (both targets)                                    |
| Connectivity     | Direct USB-Serial (`/dev/ttyUSB0`) @ 115200 baud to ROS 2 driver                                                                                    |

---

## Field Layout

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
   │  │   [ArUco #0] (50x50mm)      ┌───────────────────┐ [ArUco #1│
   │  │   (25, 425) [Row 3, Col 0]  │   xArm 1S MOUNT   │ (325, 425│
   │  │                             │ (X=175.0, Y=375.0)│ [Row 3, C│
   │  │                             │ Heading: -90.0 deg│          │
   │  │                             └───────────────────┘          │
   │  │                                                            │
   │  │            GOLDEN PICKUP ZONE (Rows 7..9, Cols 2..4)       │
   │  │                (Kinematic Sweet Spot: r = 140..260 mm)     │
   │  │             [Red & Green Targets | Nearest-First]          │
   │  │                                                            │
   │  │   [ArUco #2] (50x50mm)                            [ArUco #3] (50x50mm) │
   │  │   (25, 25) [Row 11, Col 0]                        (325, 25) [Row 11, C6│
   │  │                                                                        │
   │  │ FRONT OF WORKSPACE (Operator Position @ Row 11, Y = 0 mm)              │
   │  └────────────────────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────────> X (Width: 350 mm, Cols 0 to 6)
```

---

## Kinematics & Motion Planning

1. **Mounting Heading ($-90.0^\circ$ South)**: Base motor 6 centered at $502$ ticks faces straight down Column 3 midline. Active front arena spans only $\pm 26^\circ$ ($394\text{--}610$ ticks).
2. **Radial Reach ($R \in [160, 300\text{ mm}]$)**: High mechanical torque band; eliminates straight-arm singular extension.
3. **Nearest-First Policy**: All detected target cubes (Red and Green) are sorted in ascending radius ($r$). The closest blocks are picked first, preventing the claw or forearm from plowing through foreground objects.
4. **7-Phase Overhead Rear-Flip Trajectory**:
   - Phase 1: Azimuth Slew & Standoff Approach (hover $+50\text{ mm}$, claw pre-opened to 200 ticks)
   - Phase 2: Descent & Grasp Stance (descend to $+12.5\text{ mm}$, orient wrist roll to cube yaw $\psi$)
   - Phase 3: Gripper Clamping & Grip Verification (clamp to tick 500, verify Motor 1 position feedback in $[380, 520]$ ticks)
   - Phase 4: High Vertical Lift (lift vertically to safe transit altitude $Z = 130.0\text{ mm}$)
   - Phase 5: Overhead Rear-Facing Pitch (pitch Motors 5, 4, 3 backward overhead into rear stance without changing base yaw)
   - Phase 6: Rear Slew & Container Standoff (slew Motor 6 slightly to designated container: Red @ 650 ticks, Green @ 354 ticks, $Z = 45\text{ mm}$)
   - Phase 7: Drop & Return (open claw to 200 ticks to release cube, pitch forward overhead back to Ready standoff stance)

---

## Vision Pipeline Summary

1. Overhead camera frame captured via V4L2 with locked exposure.
2. 4 ArUco markers (`DICT_4X4_50`, IDs 0..3) detected and matched to metric destination corners $[(0,0), (600,0), (600,800), (0,800)]$.
3. Homography transforms active arena into $600 \times 800\text{ px}$ metric grid ($2.0\text{ px/mm}$).
4. Spatial exclusion mask eliminates robot chassis reflection zone (Rows 3..5, Cols 2..4).
5. HSV segmentation isolates Red targets (dual bands $[0..10]$ and $[170..180]$) and Green targets ($[38..85]$).
6. Spatial moments compute cube centroids $(X_w, Y_w)$ and yaw orientation $\psi$.
7. Detections mapped to arm cylindrical/polar frame $(r, \theta_{\text{arm}})$. Both Red and Green targets are queued for pick execution into their respective rear containers.
