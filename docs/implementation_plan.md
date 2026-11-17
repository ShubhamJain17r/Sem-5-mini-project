# Single xArm 1S Red-Object Picking Robot — Implementation Plan

## System Overview

A single LewanSoul / Hiwonder xArm 1S robotic arm picks red wooden craft blocks from a physical **$7 \times 12$ hand-drawn grid arena ($350\text{ mm} \times 600\text{ mm}$, $Z = 0.0\text{ mm}$)** and deposits them into a **dedicated Rear Flank container box** at $(300.0, 550.0, 40.0\text{ mm})$ with Motor 6 drop yaw tick $875$.

The operator views the setup from the **FRONT** (Row 11, $Y = 0.0\text{ mm}$). The entire workcell is constructed on a single **flat wooden platform** ($350\text{ mm} \text{ wide} \times 600\text{ mm} \text{ deep}$). The arm is mounted in Rows 3..5, Columns 2..4 at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm}, Z = 0.0\text{ mm})$ with nominal heading **$-35.0^\circ$** (South-East):

- **Picking Arena ($7 \times 12$ Grid Geometry)**: Hand-drawn $50.0\text{ mm} \times 50.0\text{ mm}$ square cells across 7 columns (Cols 0..6) and 12 rows (Rows 0..11). Active pick zone lies in Rows 7..10 ($r \in [160.0, 300.0\text{ mm}]$). Compliant foam tape ($8\text{--}10\text{ mm}$ tall) retains cubes within the arena.
- **Drop Zone (Rear Flank Single Container Box)**:
  - Physical container box ($100\times 100\times 35\text{ mm}$) placed in Rows 0..1, Columns 5..6 at $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 0.0\text{ mm})$.
  - Drop Target: $(X = 300.0\text{ mm}, Y = 550.0\text{ mm}, Z = 40.0\text{ mm})$, base yaw tick $875$ ($\le 879$ hardware limit). Zero 3D printing needed.
- **Color Discrimination Methodology**:
  - **Target (Red)**: Dual-band HSV ($[0..10]$ and $[170..180]$) detects red cubes. Only red objects are queued, validated, and picked.
  - **Distractor (Green)**: Detected for telemetry but marked `targetable: false`, never queued or picked.
  - **Chassis Exclusion (Blue)**: Bypassed to eliminate reflections from the blue anodized aluminum robot body.
  - **Spatial Exclusion Mask**: Rectangular mask in image space for Rows 3..5, Cols 2..4 ($X \in [100, 250]$, $Y \in [300, 450]$) zeroes out the robot base.
- **Table-Coplanar ArUco Fiducials**: 4 ArUco markers (`DICT_4X4_50`, IDs 0..3, size $50\times 50\text{ mm}$) mounted flat at $Z = 0.0\text{ mm}$ in clockwise order:
  - ID 0: $(25.0, 425.0\text{ mm})$ [Row 3, Col 0] $\to (0, 0)$
  - ID 1: $(325.0, 425.0\text{ mm})$ [Row 3, Col 6] $\to (600, 0)$
  - ID 2: $(325.0, 25.0\text{ mm})$ [Row 11, Col 6] $\to (600, 800)$
  - ID 3: $(25.0, 25.0\text{ mm})$ [Row 11, Col 0] $\to (0, 800)$
  - Rectified metric grid: $600 \times 800\text{ px}$ at $2.0\text{ px/mm}$ ($1\text{ px} = 0.5\text{ mm}$).

---

## Hardware Specifications

| Component        | Spec                                                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arm              | 1× LewanSoul / Hiwonder xArm 1S mounted at $(X = 175.0\text{ mm}, Y = 375.0\text{ mm})$, heading $-35.0^\circ$ (South-East)                       |
| Baseboard        | Platform board ($350\text{ mm} \times 600\text{ mm} \times 18\text{ mm}$) with hand-drawn $7\times 12$ grid ($5\text{ cm}$ cells)                   |
| Compute          | NVIDIA Jetson Orin Nano / Raspberry Pi 5 (Rear Bay $Y \in [500, 600\text{ mm}]$)                                                                     |
| Camera           | Logitech C270 or C920 (USB) on scrap aluminum stand @ $H \approx 620\text{ mm}$ nadir over $(175, 250\text{ mm})$                                  |
| Picking Arena    | Rows 7..10 active pick zone ($Z = 0.0\text{ mm}$, $r \in [160, 300\text{ mm}]$) with compliant foam tape perimeter                                  |
| ArUco Setup      | 4 coplanar markers (IDs 0–3, $50\times 50\text{ mm}$) in clockwise order at $(25, 425), (325, 425), (325, 25), (25, 25)$                           |
| Container Box    | Physical container box ($100\times 100\times 35\text{ mm}$) at Rear Flank $(X=300, Y=550\text{ mm})$, drop yaw tick $875$                            |
| Objects          | Wooden craft cubes ($25.4\text{ mm}$, $15\text{ g}$): Red (target), Green (distractor)                                                              |
| Connectivity     | Direct USB-Serial (`/dev/ttyUSB0`) @ 115200 baud to ROS 2 driver                                                                                    |

---

## Field Layout

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

---

## Kinematics & Motion Planning

1. **Mounting Heading ($-35.0^\circ$)**: Ensures Motor 6 (safe travel $126\text{--}879$ ticks) easily spans arena picks ($139\text{--}785$ ticks) and container drops ($875$ ticks) without crossing the hardware deadzone.
2. **Radial Reach ($R \in [160, 300\text{ mm}]$)**: High mechanical torque band; eliminates straight-arm singular extension.
3. **Nearest-First Policy**: All detected red targets are sorted in ascending radius ($r$). The closest blocks are picked first, preventing the claw or forearm from plowing through foreground objects.
4. **5-Phase Minimum-Jerk Trajectory**:
   - Phase 1: Azimuth Slew & Standoff Approach (hover $+40\text{ mm}$, claw pre-opened to 200 ticks)
   - Phase 2: Horizontal Radial Advance (descend to $+12.5\text{ mm}$, orient wrist roll to cube yaw $\psi$)
   - Phase 3: Gripper Clamping & Settling (clamp to tick 500)
   - Phase 4: Vertical Lift & Retract (lift to $+60\text{ mm}$, retract radially)
   - Phase 5: Container Swivel & Release (swivel to yaw tick $875$ at $Z=40\text{ mm}$, open claw, return home)

---

## Vision Pipeline Summary

1. Overhead camera frame captured via V4L2 with locked exposure.
2. 4 ArUco markers (`DICT_4X4_50`, IDs 0..3) detected in clockwise order.
3. Homography transforms active arena into $600 \times 800\text{ px}$ metric grid ($2.0\text{ px/mm}$).
4. Spatial exclusion mask eliminates robot chassis reflection zone (Rows 3..5, Cols 2..4).
5. HSV segmentation isolates Red targets (dual bands $[0..10]$ and $[170..180]$) and Green distractors ($[38..82]$).
6. Spatial moments compute cube centroids $(X_w, Y_w)$ and yaw orientation $\psi$.
7. Detections mapped to arm cylindrical/polar frame $(r, \theta_{\text{arm}})$. Only Red targets are queued for pick execution.
