# Dual xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from a continuous, **two-center stadium / elliptical arena** on their right flank and sort them into **two dedicated 2×2 Collection Boxes** mounted on their left flank in the **North-West (NW)** and **South-West (SW)** positions.

The operator views the setup from the **FRONT**. The two arms are aligned along a central front-to-back axis (xArm 2 in front @ $Y = 180\text{ mm}$, xArm 1 in back @ $Y = 420\text{ mm}$), spaced $240\text{ mm}$ apart along the collinear axis at $X = 360\text{ mm}$.

- **Right Side (Picking Arena)**: A continuous, two-centered elliptical/stadium workspace spanning $180^\circ\text{--}220^\circ$ around the right flank of each arm, merging seamlessly at the central partition line ($Y = 300\text{ mm}$). The arena platform is **elevated by 20 mm** and lined with a high-friction matte silicone mat ($\mu \approx 0.8$).
- **Left Side (Drop Zone — Resolving $240^\circ$ Servo ROM)**:
  - **Box 1 (North-West)**: Dedicated to Arm 1, centered at $(115\text{ mm}, 500\text{ mm})$, oriented at an azimuth of $\approx +125^\circ$.
  - **Box 2 (South-West)**: Dedicated to Arm 2, centered at $(115\text{ mm}, 100\text{ mm})$, oriented at an azimuth of $\approx -125^\circ$.
  - **$240^\circ$ ROM Compliance**: Because the LewanSoul / Hiwonder LX-15D bus servos have a mechanical limit of $240^\circ$, directly behind the arm ($180^\circ$ due West) is a physical deadzone. Angling the boxes to NW and SW limits total rotation to $\approx 205^\circ \le 240^\circ$, completely preventing servo stall.
- **Central Electronics & Power Bay**: The unutilized zone on the left flank ($X \in [55, 175\text{ mm}], Y \in [215, 385\text{ mm}]$) hosts the NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) and 7.4V / 10A dual power distribution hub. This reduces USB-serial cable runs to $< 25\text{ cm}$, eliminates RF interference near the vision plane, and provides clean power distribution.
- **Kinematic Sweet Spot**: Arms are mounted flat on the baseboard ($Z = 0\text{ mm}$), utilizing the built-in $65\text{ mm}$ height to Motor 2. Pick radius is bounded between $R_{\min} = 120\text{ mm} (d_{\text{edge}} = 5\text{ cm})$ and $R_{\max} = 320\text{ mm} (d_{\text{edge}} = 25\text{ cm})$ to eliminate joint singularities and motor stall at mechanical limits.
- **Dedicated Arena Vision Framing**: The 4 ArUco markers are placed exclusively around the **elevated picking arena** on the right (Marker 0 and Marker 2 are collinear with the central arm axis at $X = 360\text{ mm}$; Marker 1 and Marker 3 are on the outer right boundary at $X = 730\text{ mm}$). All 4 markers sit on the elevated platform ($Z = +20\text{ mm}$) to ensure strict coplanarity. The drop boxes and electronics bay on the left are completely outside the camera's warped perspective frame.

An overhead USB camera mounted on a boom stand at 65–75 cm captures the picking arena. The system runs under **ROS 2** on a NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5).

> [!NOTE]
> For in-depth mechanical and kinematic specifications, consult the dedicated stepping stone documents:
>
> - **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) — Baseboard construction, spatial dimensions, component placements, barrier profiles, lighting, and wiring bay.
> - **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) — Analytical closed-form IK ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque/friction mechanics, and 5-phase pick trajectory.
> - **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography, hardware exposure lock, HSV color segmentation, and spatial centroid extraction.
> - **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, custom interfaces, dynamic load balancing with hysteresis, spatial crossover mutex, and production Python nodes.

---

## Hardware Specifications

| Component        | Spec                                                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arms             | 2× xArm 1s (5-DOF/6-DOF bus servo arms, Motor 2 axis at $65\text{ mm}$)                                                                             |
| Mounting         | Mounted flat on main baseboard ($Z = 0\text{ mm}$) along collinear axis ($X = 360\text{ mm}$)                                                       |
| Compute          | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (located in Central Electronics Bay)                                                          |
| Camera           | Logitech C270 or C920 (USB) mounted on overhead boom at 65–75 cm                                                                                    |
| Baseboard        | $800\text{ mm} \times 600\text{ mm}$ flat rigid board                                                                                               |
| Picking Arena    | 20 mm elevated board on RIGHT side with matte silicone friction mat                                                                                 |
| ArUco Setup      | 4 markers framing the elevated arena: Markers 0 & 2 collinear with arms, Markers 1 & 3 on outer boundary (all at $Z = +20\text{ mm}$)               |
| Arena Limits     | $R_{\min} = 120\text{ mm} (d_{\text{edge}} = 5\text{ cm})$ inner barrier, $R_{\max} = 320\text{ mm} (d_{\text{edge}} = 25\text{ cm})$ outer barrier |
| Collection Boxes | 2× dedicated 2×2 boxes (Box 1 in NW @ $+125^\circ$, Box 2 in SW @ $-125^\circ$, table level $Z = 0\text{ mm}$)                                      |
| Electronics Bay  | Dedicated Central Bay on left flank ($X \in [55, 175\text{ mm}], Y \in [215, 385\text{ mm}]$)                                                       |
| Objects          | Wooden cubes (25 mm), spray-painted in 4 colors                                                                                                     |
| Colors           | Red, Green, Blue, Yellow (HSV-separated)                                                                                                            |
| Connectivity     | USB-Serial (`/dev/ttyUSB0`, `/dev/ttyUSB1`) to ROS 2 drivers                                                                                        |

---

## Field Layout

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                                                                        │
│                              ◄─── BACK OF WORKSPACE ───►                               │
│                                                                                        │
│   ┌─────────────────┐                     ┌────────┐             /───────────────────\ │
│   │  Box 1 (2×2)    │ ◄── 205° NW Drop ── │ xArm 1 │ ──────────►│  Arm 1 Arc Zone    │ │
│   │  (Arm 1 - NW)   │                     │ (Back) │    Pick    │  (Right Arena)     │ │
│   └─────────────────┘                     └────────┘             \                   / │
│                                                │  ◤ ArUco #2      │  CONTINUOUS       ││
│   ┌─────────────────┐             ═════════════╪══════════════════│- TWO-CENTER       ││
│   │  CENTRAL        │               Midline Partition (Y=300mm)   │  STADIUM ARENA    ││
│   │  ELECTRONICS    │                          │                  │  (ELEVATED 20mm)  ││
│   │  BAY: Hub & RPi │                          │                  /                   │ │
│   └─────────────────┘                     ┌────────┐             /                     │
│                                           │ xArm 2 │ ──────────►│  Arm 2 Arc Zone    │ │
│   ┌─────────────────┐ ◄── 205° SW Drop ── │(Front) │    Pick    │  (Right Arena)     │ │
│   │  Box 2 (2×2)    │                     └────────┘             \───────────────────/ │
│   │  (Arm 2 - SW)   │                            ◣ ArUco #0                  ArUco #1 ◢│
│   └─────────────────┘                                                                  │
│                              ◄─── FRONT OF WORKSPACE ───►                              │
│                                           ▲                                            │
│                                  (Operator Position)                                   │
│                                                                                        │
│   ◄─── LEFT (Drop Zone & Electronics) ───►  │  ◄─── RIGHT (Calibrated Arena) ───►      │
│   (Boxes @ Z = 0 mm, Bay @ Z = 0 mm)        │  (4 ArUco markers on Z = 20 mm)       │
└────────────────────────────────────────────────────────────────────────────────────────┘

  Collinear Axis:  ArUco #0, xArm 2, xArm 1, and ArUco #2 lie along X = 360 mm
  Vision Focus:    100% of camera pixels warped onto the picking arena
  Zero Occlusion:  Drop boxes and Central Electronics Bay are completely excluded from vision
```

---

## Kinematics & Elevation Optimization

1. **Shoulder Height Leverage**: The arm's Motor 2 (shoulder pitch axis) is already **$65\text{ mm}$** off the ground.
2. **Elevated Arena ($+20\text{ mm}$)**: Raising the picking arena floor by $20\text{ mm}$ places the cube center at $Z \approx 32.5\text{ mm}$. The vertical drop from Motor 2 is only $\approx 35\text{ mm}$ instead of $52.5\text{ mm}$. The arm reaches horizontally in its maximum mechanical torque band without scraping the table.
3. **Radial Boundary ($R_{\max} = 320\text{ mm} \iff d_{\text{edge}} = 25\text{ cm}$)**: Bounding the maximum reach at $25\text{ cm}$ from the baseplate edge ($32\text{ cm}$ from center) backs off $2.4\text{ cm}$ before mechanical lockup ($27.4\text{ cm}$), maintaining a healthy $\approx 122^\circ$ elbow angle to prevent motor stalling.
4. **Coplanar Marker Alignment**: Mounting all 4 ArUco markers directly on the raised arena ($Z = +20\text{ mm}$) guarantees zero perspective tilt error during homography calculation.
5. **$240^\circ$ Base Servo ROM Optimization**: Placing Box 1 in the North-West ($\approx +125^\circ$) and Box 2 in the South-West ($\approx -125^\circ$) keeps total base yaw swing to $\approx 205^\circ \le 240^\circ$, avoiding the $180^\circ$ physical deadzone directly behind the arm.
6. **Full Analytical Inverse Kinematics (IK) & Grasp Planning**: See the dedicated mathematical and mechanical architecture document [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) for closed-form joint angle equations ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque models, and 5-phase trajectory waypoint generation.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```text
Frame from camera (Overhead @ 65–75 cm above arena)
  → Detect 4 ArUco markers framing the elevated arena:
      - ArUco #0: Front-Center (X = 360 mm, Y = 55 mm, collinear with arms)
      - ArUco #1: Front-Right  (X = 730 mm, Y = 55 mm)
      - ArUco #2: Back-Center  (X = 360 mm, Y = 545 mm, collinear with arms)
      - ArUco #3: Back-Right   (X = 730 mm, Y = 545 mm)
  → Compute perspective transform → rectified top-down view (dedicated arena mm plane)
  → Convert rectified image to HSV
  → Threshold for 4 colors (Red, Green, Blue, Yellow) → binary masks
  → Find contours per mask → filter by area (25 mm cube profile)
  → Compute centroid (pixel) per detected cube
  → Map to arena mm: (X_arena, Y_arena)
  → Convert to Arm Polar Frame relative to respective arm base:
      ΔX = X_arena - X_base,  ΔY = Y_arena - Y_base
      r = sqrt(ΔX² + ΔY²),    θ = atan2(ΔY, ΔX)
  → Boundary check: Verify 120 mm ≤ r ≤ 320 mm (5 cm ≤ d_edge ≤ 25 cm) within right-flank arc
  → Output: [{color, x_mm, y_mm, r_mm, theta_deg, target_arm}, ...]
```

---

## ROS 2 Architecture & Coordination

_(Full mathematical derivations, QoS profiles, mutex state machines, launch scripts, and Python implementations detailed in [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md))_:

### Nodes

| Node             | Role                                                                                                                                 |             Rate              |
| ---------------- | ------------------------------------------------------------------------------------------------------------------------------------ | :---------------------------: |
| `camera_node`    | Acquires frames from `/dev/video0` with exposure lock ($156$) and WB lock ($4500\text{K}$) → `/camera/image_raw`                     |        $30\text{ Hz}$         |
| `vision_node`    | Executes 4-point ArUco homography warp + HSV color segmentation → `/vision/detected_objects`                                         |   $10\text{--}15\text{ Hz}$   |
| `scheduler_node` | Dynamic load balancing across $Y = 300\text{ mm}$ midline, crossover mutex arbitration ($Y \in [250, 350\text{ mm}]$), task dispatch |      $20\text{ Hz}$ loop      |
| `arm1_node`      | North manipulator controller: closed-form analytical IK, 5-phase trajectory profiling, serial packet writer (`/dev/xarm_arm1`)       | Event + $50\text{ Hz}$ status |
| `arm2_node`      | South manipulator controller: closed-form analytical IK, 5-phase trajectory profiling, serial packet writer (`/dev/xarm_arm2`)       | Event + $50\text{ Hz}$ status |
| `dashboard_node` | System telemetry, cycle times, sorting counts, and diagnostics via Foxglove Studio / Web UI                                          |         $5\text{ Hz}$         |

### Custom Message & Service Interfaces

```text
DetectedObject.msg      → uint32 object_id, string color, float64 x_mm, float64 y_mm, float64 z_mm, float64 yaw_deg, float64 confidence
DetectedObjectArray.msg  → std_msgs/Header header, DetectedObject[] objects, uint32 total_detected, bool homography_locked
ArmTask.msg             → uint32 task_id, string arm_id, DetectedObject target_object, float64 target_r_mm, float64 target_theta_deg, string destination_box, int32 compartment_index, float64 drop_theta_deg, bool requires_mutex
ArmStatus.msg           → std_msgs/Header header, string arm_id, string state, uint32 active_task_id, bool is_busy, bool has_cube, bool holds_mutex, float64[5] joint_angles_deg, float64 bus_voltage_v, string error_code
AcquireZone.srv         → Request(arm_id, zone_id, timeout_sec) / Response(bool success, string message)
ReleaseZone.srv         → Request(arm_id, zone_id) / Response(bool success)
```

---

## Arm Motion Sequence (per task)

### Distance-Based Pitch Lookup Table (LUT)

_(Full closed-form equations and empirical calibration detailed in [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md))_:

- **Near** ($r \in [120, 170\text{ mm}] \iff d_{\text{edge}} \in [5, 10\text{ cm}]$): Steep downward approach ($\phi \in [-75^\circ, -60^\circ]$, Object-Aligned Roll $\theta_5$)
- **Mid** ($r \in [170, 250\text{ mm}] \iff d_{\text{edge}} \in [10, 18\text{ cm}]$): Angled approach ($\phi \in [-50^\circ, -25^\circ]$, Object-Aligned Roll $\theta_5$)
- **Far Boundary** ($r \in [250, 320\text{ mm}] \iff d_{\text{edge}} \in [18, 25\text{ cm}]$): Shallow horizontal approach ($\phi \in [-15^\circ, -5^\circ]$, Locked Level Roll $\theta_5 = 0^\circ$)

### Motion Execution Steps

```text
1. rotate_base(theta)            → rotate arm base to target azimuth on RIGHT arena
2. lookup_pitch(r)               → query pitch angle α from LUT for radius r (or analytical IK)
3. set_roll(0°)                  → lock wrist roll level with table
4. move_standoff(r - 30mm, z)    → position gripper backed off radially along approach vector
5. open_gripper(55mm)            → fully open 2-finger claw
6. advance_radially(r, pick_z)   → slide forward onto raised arena surface (Z ≈ 32.5 mm)
7. close_gripper()               → grip cube (mechanical self-centering against friction mat)
8. check_grip()                  → read servo position feedback; retry if failed
9. ascend_and_retract()          → lift slightly and retract radially
10. retract_to_compact()         → tuck arm into compact travel pose
11. rotate_to_drop_box()         → swing 205° CCW (Arm 1 → Box 1 NW) or CW (Arm 2 → Box 2 SW)
12. extend_to_bin(bin_x, bin_y)  → position over target color cell
13. descend(bin_z)               → lower into cell
14. open_gripper()               → release cube
15. ascend()                     → clear box rim
16. return_home()                → ready pose
```

---

## Scheduler: Mutex & Pipelined Execution

- **`arena_mutex`**: Only one arm picks in the right-side arena at any moment.
- **Independent Drop Zones**: Arm 1 and Arm 2 drop into separate NW/SW boxes on their respective flanks, completely eliminating drop collisions (**no `box_mutex` required**).

**Pipeline Sequence**:

```text
xArm 2 (Front) picks in arena (holds arena_mutex)
  → xArm 2 finishes pick, releases arena_mutex → starts 205° CW swing to Box 2 (SW)
  → simultaneously, xArm 1 (Back) acquires arena_mutex → picks in arena
  → simultaneously, xArm 2 drops in private Box 2 (no waiting!)
  → xArm 1 finishes pick, releases arena_mutex → starts 205° CCW swing to Box 1 (NW)
  → xArm 2 re-acquires arena_mutex for next pick
```

---

## Collection Box Coordinates (`config/bin_coordinates.yaml`)

Two separate 2×2 boxes located on the left flank of each arm at $Z = 0\text{ mm}$:

```text
     Box 1 (North-West: Arm 1)             Box 2 (South-West: Arm 2)
┌───────────┬───────────┐             ┌───────────┬───────────┐
│ Red (R)   │ Green (G) │             │ Red (R)   │ Green (G) │
├───────────┼───────────┤             ├───────────┼───────────┤
│ Blue (B)  │ Yellow (Y)│             │ Blue (B)  │ Yellow (Y)│
└───────────┴───────────┘             └───────────┴───────────┘
```

| Box / Cell         |         World Coordinates $(X, Y, Z)$         | Target Color | Drop Method           |
| ------------------ | :-------------------------------------------: | :----------: | --------------------- |
| **Box 1 — Cell 1** | $(85\text{ mm}, 530\text{ mm}, 0\text{ mm})$  |     Red      | Top-down gravity drop |
| **Box 1 — Cell 2** | $(145\text{ mm}, 530\text{ mm}, 0\text{ mm})$ |    Green     | Top-down gravity drop |
| **Box 1 — Cell 3** | $(85\text{ mm}, 470\text{ mm}, 0\text{ mm})$  |     Blue     | Top-down gravity drop |
| **Box 1 — Cell 4** | $(145\text{ mm}, 470\text{ mm}, 0\text{ mm})$ |    Yellow    | Top-down gravity drop |
| **Box 2 — Cell 1** | $(85\text{ mm}, 130\text{ mm}, 0\text{ mm})$  |     Red      | Top-down gravity drop |
| **Box 2 — Cell 2** | $(145\text{ mm}, 130\text{ mm}, 0\text{ mm})$ |    Green     | Top-down gravity drop |
| **Box 2 — Cell 3** |  $(85\text{ mm}, 70\text{ mm}, 0\text{ mm})$  |     Blue     | Top-down gravity drop |
| **Box 2 — Cell 4** | $(145\text{ mm}, 70\text{ mm}, 0\text{ mm})$  |    Yellow    | Top-down gravity drop |
