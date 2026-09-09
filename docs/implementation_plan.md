# Dual xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from a continuous, **two-center stadium / elliptical arena** on their right flank and sort them into **two dedicated 2×2 Collection Boxes** mounted on their left flank.

The operator views the setup from the **FRONT**. The two arms are aligned along a central front-to-back axis (xArm 2 in front, xArm 1 in back), spaced ~240–300 mm apart.

- **Right Side (Picking Arena)**: A continuous, two-centered elliptical/stadium workspace spanning $180^\circ\text{--}220^\circ$ around the right flank of each arm, merging seamlessly at the central partition line. The arena platform is **elevated by 15–20 mm** and lined with a high-friction matte silicone mat.
- **Left Side (Drop Zone)**: Two separate 2×2 collection boxes mounted on the left flank of each arm at baseboard level ($0\text{ mm}$), completely eliminating drop contention.
- **Motion**: Arms reach RIGHT to pick, then rotate $\approx 180^\circ$ LEFT to drop.
- **Kinematic Sweet Spot**: Arms are mounted flat on the baseboard (utilizing the built-in $65\text{ mm}$ height to Motor 2). Pick radius is bounded between $R_{\min} = 100\text{ mm}$ and $R_{\max} = 240\text{ mm}$ to eliminate joint singularities and motor stall at mechanical limits.
- **Dedicated Arena Vision Framing**: The 4 ArUco markers are placed exclusively around the **elevated picking arena** on the right (Marker 0 and Marker 2 are collinear with the central arm axis; Marker 1 and Marker 3 are on the outer right boundary). All 4 markers sit on the elevated platform ($Z = 15\text{--}20\text{ mm}$) to ensure strict coplanarity. The drop boxes on the left are completely outside the camera's warped perspective frame.

An overhead USB camera mounted on a boom stand at 60–75 cm captures the picking arena. The system runs under **ROS 2** on a Jetson Nano or Raspberry Pi 5.

---

## Hardware

| Component | Spec |
|---|---|
| Arms | 2× xArm 1s (5-DOF/6-DOF bus servo arms, Motor 2 axis at $65\text{ mm}$) |
| Mounting | Mounted flat on main baseboard ($Z = 0\text{ mm}$) |
| Compute | Jetson Nano / Raspberry Pi 5 |
| Camera | Logitech C270 or C920 (USB) mounted on overhead boom at 60–75 cm |
| Baseboard | 80 cm × 60 cm flat board |
| Picking Arena | 15–20 mm elevated board on RIGHT side with matte silicone friction mat |
| ArUco Setup | 4 markers framing the elevated arena: Markers 0 & 2 in line with arms, Markers 1 & 3 on right edge |
| Arena Limits | $R_{\min} = 100\text{ mm}$ inner barrier, $R_{\max} = 240\text{ mm}$ outer barrier |
| Collection Boxes | 2× dedicated 2×2 boxes (one per arm, mounted on LEFT side at $Z = 0\text{ mm}$) |
| Objects | Wooden cubes (25 mm), spray-painted in 4 colors |
| Colors | Red, Green, Blue, Yellow (HSV-separated) |
| Connectivity | USB-Serial (preferred) or WiFi to ROS 2 driver |

---

## Field Layout

```
┌────────────────────────────────────────────────────────────────────────┐
│                                                                        │
│                      ◄─── BACK OF WORKSPACE ───►                       │
│                                                                        │
│   ┌─────────────┐             ┌────────┐         /───────────────────\ │
│   │ Box 1 (2×2) │ ◄── 180° ── │ xArm 1 │ ──────►│  Arm 1 Arc Zone   │ │
│   │  (Arm 1)    │    Drop     │ (Back) │  Pick  │  (180°–220° Right)│ │
│   └─────────────┘             └────────┘         \                   / │
│                                   │  ◤ ArUco #2   │  CONTINUOUS       ││
│                        Midline Partition Line - - │- TWO-CENTER       ││
│                                   │               │  STADIUM ARENA    ││
│   ┌─────────────┐             ┌────────┐         /   (ELEVATED 15mm) │ │
│   │ Box 2 (2×2) │ ◄── 180° ── │ xArm 2 │ ──────►│  Arm 2 Arc Zone   │ │
│   │  (Arm 2)    │    Drop     │(Front) │  Pick  │  (180°–220° Right)│ │
│   └─────────────┘             └────────┘         \───────────────────/ │
│                                      ◣ ArUco #0              ArUco #1 ◢│
│                      ◄─── FRONT OF WORKSPACE ───►                      │
│                                ▲                                       │
│                       (Operator Position)                              │
│                                                                        │
│   ◄─── LEFT (Drop Zone) ───►  │  ◄─── RIGHT (Calibrated Arena) ───►    │
│   (Boxes sit at Z = 0 mm)     │  (4 ArUco markers on Z = 15-20mm)      │
└────────────────────────────────────────────────────────────────────────┘

  Collinear Axis:  ArUco #0, xArm 2, xArm 1, and ArUco #2 lie along the inner edge
  Vision Focus:    100% of camera pixels warped onto the picking arena
  Zero Occlusion:  Drop boxes are completely excluded from the vision homography
```

---

## Kinematics & Elevation Optimization

1. **Shoulder Height Leverage**: The arm's Motor 2 (shoulder pitch axis) is already **$65\text{ mm}$** off the ground.
2. **Elevated Arena ($+15\text{--}20\text{ mm}$)**: Raising the picking arena floor by $15\text{--}20\text{ mm}$ places the cube center at $Z \approx 30\text{ mm}$. The vertical drop from Motor 2 is only $\approx 35\text{ mm}$ instead of $52.5\text{ mm}$. The arm reaches horizontally in its maximum mechanical torque band without scraping the table.
3. **Radial Boundary ($R_{\max} = 240\text{ mm}$)**: Bounding the maximum reach at $24\text{ cm}$ (instead of $26\text{--}27\text{ cm}$) keeps the elbow joint at $\approx 150^\circ$, completely preventing joint singularity, servo stutter, and motor stalling.
4. **Coplanar Marker Alignment**: Mounting all 4 ArUco markers directly on the raised arena ($Z = 15\text{--}20\text{ mm}$) guarantees zero perspective tilt error during homography calculation.
5. **Gravity-Assisted Drops**: The collection boxes on the left remain at table level ($0\text{ mm}$), allowing the arm to drop cubes into the bins from above effortlessly.
6. **Full Analytical Inverse Kinematics (IK) & Grasp Planning**: See the dedicated mathematical and mechanical architecture document [`docs/kinematics_and_grasp_planning.md`](file:///d:/Sem-3-mini-project/docs/kinematics_and_grasp_planning.md) for closed-form joint angle equations ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque models, and 5-phase trajectory waypoint generation.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```
Frame from camera (Overhead @ 60–75 cm above arena)
  → Detect 4 ArUco markers framing the elevated arena:
      - ArUco #0: Front-Center (in line with arms)
      - ArUco #1: Front-Right
      - ArUco #2: Back-Center (in line with arms)
      - ArUco #3: Back-Right
  → Compute perspective transform → rectified top-down view (dedicated arena mm plane)
  → Convert rectified image to HSV
  → Threshold for 4 colors (Red, Green, Blue, Yellow) → binary masks
  → Find contours per mask → filter by area (25 mm cube profile)
  → Compute centroid (pixel) per detected cube
  → Map to arena mm: (X_arena, Y_arena)
  → Convert to Arm Polar Frame relative to respective arm base:
      ΔX = X_arena - X_base,  ΔY = Y_arena - Y_base
      r = sqrt(ΔX² + ΔY²),    θ = atan2(ΔY, ΔX)
  → Boundary check: Verify 100 mm ≤ r ≤ 240 mm within right-flank arc
  → Output: [{color, x_mm, y_mm, r_mm, theta_deg, target_arm}, ...]
```

---

## ROS 2 Architecture

### Nodes

| Node | Role |
|---|---|
| `camera_node` | Publishes raw frames from USB webcam → `/camera/image_raw` |
| `vision_node` | On trigger: runs ArUco homography + HSV pipeline on right arena → publishes `/detected_objects` |
| `scheduler_node` | Splits cubes by front/back midline, assigns to Arm 1 / Arm 2, manages `arena_mutex` |
| `arm_node` | Executes task queue for one arm using Radial LUT approach (two instances: `arm_1`, `arm_2`) |
| `dashboard_node` | Runs rosbridge WebSocket server + HTTP server for web UI |

### Custom Messages

```
DetectedObject.msg    → string color, float32 x_mm, float32 y_mm, float32 r_mm, float32 theta_deg, string assigned_arm
ArmTask.msg           → string color, float32 pick_r, float32 pick_theta, float32 pick_z,
                         float32 bin_x, float32 bin_y, float32 bin_z, string arm_id
ArmStatus.msg         → string arm_id, string state, int32 tasks_remaining, bool grip_ok
```

---

## Arm Motion Sequence (per task)

### Distance-Based Pitch Lookup Table (LUT)
- **Near** ($r \approx 10\text{--}14\text{ cm}$): Steep downward pitch ($\approx 40^\circ\text{--}45^\circ$)
- **Mid** ($r \approx 15\text{--}19\text{ cm}$): Moderate approach pitch ($\approx 25^\circ\text{--}30^\circ$)
- **Far** ($r \approx 20\text{--}24\text{ cm}$): Shallow horizontal pitch ($\approx 15^\circ\text{--}20^\circ$)

### Motion Execution Steps:
```
1. rotate_base(theta)            → rotate arm base to target azimuth on RIGHT side
2. lookup_pitch(r)               → query pitch angle α from LUT for radius r
3. set_roll(0°)                  → lock wrist roll level with table
4. move_standoff(r - 30mm, z)    → position gripper backed off radially along approach vector
5. open_gripper(55mm)            → fully open 2-finger claw
6. advance_radially(r, pick_z)   → slide forward onto raised arena surface (Z ≈ 30 mm)
7. close_gripper()               → grip cube (mechanical self-centering against friction mat)
8. check_grip()                  → read servo position; retry if failed
9. ascend_and_retract()          → lift slightly and retract radially
10. retract_to_compact()         → tuck arm into compact travel pose
11. rotate_left_180()            → swing 180° from RIGHT arena to LEFT dedicated 2×2 box
12. extend_to_bin(bin_x, bin_y)  → position over target color cell
13. descend(bin_z)               → lower into cell
14. open_gripper()               → release cube
15. ascend()                     → clear box rim
16. return_home()                → ready pose
```

---

## Scheduler: Mutex & Pipelined Execution

- **`arena_mutex`**: Only one arm picks in the right-side arena at any moment.
- **Independent Drop Zones**: Arm 1 and Arm 2 drop into separate boxes on their respective left sides, completely eliminating drop collisions (**no `box_mutex` required**).

**Pipeline Sequence**:
```
xArm 2 (Front) picks in arena (holds arena_mutex)
  → xArm 2 finishes pick, releases arena_mutex → starts 180° swing to Box 2
  → simultaneously, xArm 1 (Back) acquires arena_mutex → picks in arena
  → simultaneously, xArm 2 drops in private Box 2 (no waiting!)
  → xArm 1 finishes pick, releases arena_mutex → starts 180° swing to Box 1
  → xArm 2 re-acquires arena_mutex for next pick
```

---

## Collection Box Coordinates (`config/bin_coordinates.yaml`)

Two separate 2×2 boxes located on the left flank of each arm at $Z = 0\text{ mm}$:

```
      Box 1 (Left of Arm 1)                 Box 2 (Left of Arm 2)
┌───────────┬───────────┐             ┌───────────┬───────────┐
│ Red (R)   │ Green (G) │             │ Red (R)   │ Green (G) │
├───────────┼───────────┤             ├───────────┼───────────┤
│ Blue (B)  │ Yellow (Y)│             │ Blue (B)  │ Yellow (Y)│
└───────────┴───────────┘             └───────────┴───────────┘
```
