# Dual xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from a **flat rectangular arena ($400\times 220\text{ mm}$, $Z = 0.0\text{ mm}$)** and sort them into **two dedicated 3D-printed 2×2 Collection Trays** mounted on their immediate lateral flanks.

The operator views the setup from the **FRONT**. The entire workcell is constructed on a single, store-cut **flat wooden baseboard** ($560\text{ mm} \text{ wide} \times 520\text{ mm} \text{ deep} \times 18\text{ mm}$ MDF). The two arms are mounted flat at $Z = 0.0\text{ mm}$ spaced $220\text{ mm}$ apart center-to-center, with Motor 6 oriented at inward bisector headings ($-110.0^\circ$ for Arm 1, $-70.0^\circ$ for Arm 2) toward the arena midline ($X = 280\text{ mm}$).

- **Picking Arena (Flat Tabletop Geometry)**: Sits directly flat on the wooden baseboard at $Z = 0.0\text{ mm}$ ($400\text{ mm}$ wide $\times 220\text{ mm}$ depth, $X \in [80, 480]$, $Y \in [40, 260]$). Retained by a soft compliant **$10\text{ mm}$ self-adhesive foam tape perimeter** to prevent cubes from rolling off. The intersecting radial arcs of both inward-angled arms create an active **central cooperative overlap zone** ($X \in [240, 320\text{ mm}]$) with zero unreachable blind spots.
- **Drop Zone (Immediate Flank 2×2 Trays)**:
  - **Box 1 (Left Flank)**: Dedicated to Arm 1, centered at $(50, 360, 0)$ with 4 color compartments (Red, Green, Blue, Yellow).
  - **Box 2 (Right Flank)**: Dedicated to Arm 2, centered at $(510, 360, 0)$ with 4 matching color compartments.
  - **Drop Cycle Optimization**: Relocating drop boxes to the immediate flanks slashes the drop swing from $125^\circ$ down to a fast **$\approx 70.0^\circ$ swing**, reducing cycle time by $>50\%$ and eliminating LX-15D base servo stall.
- **Rear Electronics & Power Bay**: The area behind the arm bases ($X \in [230, 330\text{ mm}], Y \in [440, 500\text{ mm}]$) hosts the NVIDIA Jetson Orin Nano / Raspberry Pi 5 and dual 7.4V / 5A buck power hub. Surface adhesive cable clips route USB serial and 7.4V power cleanly to each arm.
- **Kinematic Sweet Spot**: Arms are mounted flat at $Z = 0\text{ mm}$, utilizing the built-in $65\text{ mm}$ height to Motor 2. Pick radius is bounded between $R_{\min} = 120\text{ mm}$ and $R_{\max} = 320\text{ mm}$, maintaining healthy elbow pitch angles ($\approx 120^\circ$) to eliminate joint singularities and motor stall.
- **Coplanar Board-Mounted ArUco Fiducials**: 4 ArUco markers (IDs 0–3, $30\times 30\text{ mm}$) printed on matte vinyl are applied flat to the wooden board at the four arena corners at $Z = 0.0\text{ mm}$, ensuring zero parallax error during planar homography.

An overhead USB camera mounted on a rear boom stand at 65–75 cm captures the picking arena. The system runs under **ROS 2** on a NVIDIA Jetson Orin Nano / Raspberry Pi 5.

> [!NOTE]
> For in-depth mechanical, kinematic, optical, and diagnostic specifications, consult the dedicated stepping stone documents:
>
> - **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container workflow, and ROS 2 Humble package development standards.
> - **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) — 4-tile modular 3D-printable base, V-configuration mounting, chamfered hexagonal arena, flank drop wells, and wiring conduits.
> - **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) — Analytical closed-form IK ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque/friction mechanics, and 5-phase pick trajectory.
> - **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography, hardware exposure lock, HSV color segmentation, and spatial centroid extraction.
> - **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, custom interfaces, dynamic load balancing with hysteresis, spatial crossover mutex, and production Python nodes.
> - **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol, Arduino Uno R4 WiFi test bench, servo deviation zeroing, and tick-to-angle mapping.
> - **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 6 grip verification, empirical Z-sag compensation, and lens undistortion.

---

## Hardware Specifications

| Component        | Spec                                                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arms             | 2× xArm 1s in V-Configuration ($-110.0^\circ$ & $-70.0^\circ$ bisector headings, $S = 220\text{ mm}$ spacing)                                      |
| Baseboard        | Single store-cut MDF board ($560\text{ mm} \times 520\text{ mm} \times 18\text{ mm}$, zero woodworking cuts)                                      |
| Compute          | NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5) (Rear Bay $Y \in [440, 500\text{ mm}]$, adhesive standoffs)                                |
| Camera           | Logitech C270 or C920 (USB) mounted on rear boom stand at 65–70 cm                                                                                  |
| Picking Arena    | Flat tabletop deck ($Z = 0.0\text{ mm}$, $400\times 220\text{ mm}$) with $10\text{ mm}$ foam retaining tape perimeter                               |
| ArUco Setup      | 4 coplanar markers (IDs 0–3, $30\times 30\text{ mm}$) applied flat at arena corners at $Z = 0.0\text{ mm}$                                          |
| Arena Limits     | $X \in [80, 480\text{ mm}]$, $Y \in [40, 260\text{ mm}]$, $R \in [110, 320\text{ mm}]$ (100% reachable)                                             |
| Collection Boxes | 2× 3D-printed 2x2 flank trays ($80\times 80\times 35\text{ mm}$, 4 color slots) at Left $(50, 360)$ and Right $(510, 360)$                          |
| Electronics Bay  | Rear Bay ($X \in [230, 330\text{ mm}], Y \in [440, 500\text{ mm}]$), surface adhesive cable clips                                                   |
| Objects          | Wooden cubes (25 mm, $15\text{ g}$), spray-painted in 4 high-contrast matte colors                                                                  |
| Colors           | Red, Green, Blue, Yellow (HSV-separated)                                                                                                            |
| Connectivity     | USB-Serial (`/dev/ttyUSB0`, `/dev/ttyUSB1`) to ROS 2 drivers via direct surface wiring                                                             |

---

## Field Layout

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                                                                        │
│                              ◄─── REAR OF WORKSPACE ───►                               │
│                                                                                        │
│                           [Overhead Camera Boom Mast Anchor]                           │
│                     ┌────────────────────────────────────────────┐                     │
│                     │         CENTRAL ELECTRONICS WEDGE          │                     │
│                     │      (Jetson Orin / RPi 5 + Buck Hub)      │                     │
│                     └─────────────┬────────────────┬─────────────┘                     │
│                                   │                │                                   │
│   ┌─────────────────┐             ▼                ▼             ┌─────────────────┐   │
│   │  Box 1 Nest     │          xArm 1            xArm 2          │  Box 2 Nest     │   │
│   │  (Left Flank)   │       (Angled +35°)     (Angled -35°)      │  (Right Flank)  │   │
│   └─────────────────┘      \                      /              └─────────────────┘   │
│       ◤ ArUco #0            \                    /                     ArUco #1 ◥      │
│                              \    CHAMFERED     /                                      │
│                               │   HEXAGONAL    │                                       │
│                               │     ARENA      │ (Elevated +25 mm)                     │
│                               │   (Overlap)    │                                       │
│                               /                \                                       │
│                              /                  \                                      │
│       ◣ ArUco #2 ───────────┴────────────────────┴─────────── ArUco #3 ◢               │
│                                                                                        │
│                             ◄─── FRONT OF WORKSPACE ───►                               │
│                                          ▲                                             │
│                                 (Operator Position)                                    │
└────────────────────────────────────────────────────────────────────────────────────────┘

  Base Topology:   4-Tile modular 3D-printed interlocking base (<= 250x250 mm tiles)
  Vision Focus:    100% of camera pixels warped onto the chamfered picking arena
  Drop Travel:     Quick ~65°-75° outward swing to lateral nested collection wells
```

---

## Kinematics & Elevation Optimization

1. **Shoulder Height Leverage**: The arm's Motor 2 (shoulder pitch axis) is **$65.0\text{ mm}$** above the baseboard surface.
2. **Flat Baseboard Arena ($Z = 0.0\text{ mm}$)**: The picking arena floor sits flat on the wooden baseboard, placing the cube center at **$Z \approx 12.5\text{ mm}$**. The vertical drop from Motor 2 is $\approx 52.5\text{ mm}$. The arm approaches horizontally or at shallow pitch in its strong mechanical torque band.
3. **Radial Boundary ($R_{\max} = 320\text{ mm} \iff d_{\text{edge}} = 25\text{ cm}$)**: Bounding the maximum reach at $25\text{ cm}$ from the baseplate edge ($32\text{ cm}$ from center) backs off $2.4\text{ cm}$ before mechanical lockup ($34.4\text{ cm}$), maintaining a healthy $\approx 122^\circ$ elbow angle to prevent motor stalling.
4. **Strict Coplanar Marker Alignment**: Mounting all 4 ArUco markers flat on the baseboard ($Z = 0.0\text{ mm}$) guarantees zero elevation perspective distortion during planar homography calculation.
5. **Immediate Flank Drop Swings ($\approx 70.0^\circ$)**: Placing 2×2 grid sorting trays on the lateral flanks of each arm (Left Tray at $(50, 360)$ for Arm 1, Right Tray at $(510, 360)$ for Arm 2) slashes base yaw travel from the old $125^\circ$ swing to only $\approx 70^\circ$, doubling throughput and reducing motor wear.
6. **Full Analytical Inverse Kinematics (IK) & Grasp Planning**: See the dedicated mathematical and mechanical architecture document [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) for closed-form joint angle equations ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque models, and 5-phase trajectory waypoint generation.
7. **Nearest-First Pick Policy (Corridor Collision Avoidance)**: To prevent horizontal claw approaches from plowing through foreground cubes, the queue is strictly sorted by ascending radial distance $r$. See [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).
8. **Empirical Z-Sag Compensation**: To counteract mechanical gear backlash and linkage flex at extreme reach ($r > 180\text{ mm}$), commanded elevation incorporates a linear sag offset $Z_{\text{cmd}} = Z_{\text{target}} + k_{\text{sag}} \cdot (r - 180)$.
9. **Software Grip Confirmation via Motor 1 (Claw / $\theta_6$) Readback**: Post-clamping position readback ($< 220\text{ ticks} = \text{miss}$, $380\text{--}520\text{ ticks} = \text{cube grasped}$) prevents empty drop swings.

> [!NOTE]
> **Kinematic Joint Indices vs. Physical Hardware Motor IDs**:
> In mathematical kinematics ($\theta_1\text{--}\theta_6$), joints are indexed base-to-tip ($\theta_1 = \text{Base Yaw}, \theta_2 = \text{Shoulder Pitch}, \dots, \theta_6 = \text{Claw Clamp}$).
> On the physical **Hiwonder xArm 1s** bus servo daisy chain, hardware IDs are addressed tip-to-base: Hardware ID 1 = Claw Gripper, ID 2 = Wrist Roll, ID 3 = Wrist Pitch, ID 4 = Elbow Pitch, ID 5 = Shoulder Pitch, ID 6 = Base Yaw. Both `config/dual_xarm_hardware.yaml` and the ROS 2 driver explicitly preserve this mapping.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```text
Frame from camera (Overhead @ 65–70 cm above arena, nadir @ X=280, Y=200)
  → Intrinsic Lens Undistortion: cv2.undistort() with camera matrix K and distortion vector D
  → Detect 4 ArUco markers framing the flat arena (Z = 0.0 mm):
      - ArUco #0: Rear-Left   (X = 85.0 mm, Y = 345.0 mm)
      - ArUco #1: Rear-Right  (X = 475.0 mm, Y = 345.0 mm)
      - ArUco #2: Front-Left  (X = 85.0 mm, Y = 55.0 mm)
      - ArUco #3: Front-Right (X = 475.0 mm, Y = 55.0 mm)
  → Compute & freeze perspective transform → rectified top-down view (dedicated arena mm plane)
  → Convert rectified image to HSV
  → Threshold for 4 colors (Red, Green, Blue, Yellow) → binary masks
  → Find contours per mask → filter by area (25 mm cube profile)
  → Compute centroid (pixel) per detected cube
  → Map to arena mm: (X_arena, Y_arena)
  → Convert to Arm Polar Frame relative to respective arm base:
      ΔX = X_arena - X_base,  ΔY = Y_arena - Y_base
      r = sqrt(ΔX² + ΔY²),    θ = atan2(ΔY, ΔX) - θ_heading
  → Boundary check: Verify 130 mm ≤ r ≤ 320 mm within arm operating sector
  → Nearest-First Sort: Order queue by ascending r
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
| `scheduler_node` | Dynamic load balancing across $X = 280\text{ mm}$ midline, crossover mutex arbitration ($X \in [240, 320\text{ mm}]$), task dispatch |      $20\text{ Hz}$ loop      |
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
6. advance_radially(r, pick_z)   → slide forward onto flat arena surface (Z ≈ 12.5 mm)
7. close_gripper()               → grip cube (mechanical self-centering against friction mat)
8. check_grip()                  → read servo position feedback; retry if failed
9. ascend_and_retract()          → lift slightly and retract radially
10. retract_to_compact()         → tuck arm into compact travel pose
11. rotate_to_drop_box()         → swing ~70° (Arm 1 → Left Flank Tray 1 @ X=50, Arm 2 → Right Flank Tray 2 @ X=510)
12. extend_to_bin(bin_x, bin_y)  → position over target color cell
13. descend(bin_z)               → lower into cell (Z ≈ 50 mm)
14. open_gripper()               → release cube
15. ascend()                     → clear box rim
16. return_home()                → ready pose
```

---

## Scheduler: Mutex & Pipelined Execution

- **`arena_mutex`**: Only one arm enters the central cooperative crossover zone ($X \in [220, 340\text{ mm}]$) at any moment. Outer quadrant picks ($X < 220\text{ mm}$ for Arm 1, $X > 340\text{ mm}$ for Arm 2) execute fully concurrently with zero mutex blocking.
- **Independent Flank Drop Trays**: Arm 1 and Arm 2 drop into separate 2×2 grid sorting trays on their respective lateral flanks (Left Flank $(50, 360)$ and Right Flank $(510, 360)$), completely eliminating drop collisions (**no `box_mutex` required**).

**Pipeline Sequence**:

```text
xArm 2 (Right) picks in arena (holds crossover mutex if crossing X=340 mm)
  → xArm 2 finishes pick, releases mutex → starts ~70° outward swing to Tray 2 (Right Flank)
  → simultaneously, xArm 1 (Left) acquires mutex → picks in crossover zone
  → simultaneously, xArm 2 drops in private Tray 2 (no waiting!)
  → xArm 1 finishes pick, releases mutex → starts ~70° outward swing to Tray 1 (Left Flank)
  → xArm 2 re-acquires mutex for next pick
```

---

## Collection Box Coordinates (`config/dual_xarm_hardware.yaml`)

Two separate 3D-printed 2×2 sorting trays ($80\times 80\times 35\text{ mm}$) located on the lateral flanks of each arm:

```text
     Tray 1 (Left Flank: Arm 1 @ 170, 360)       Tray 2 (Right Flank: Arm 2 @ 390, 360)
         Center @ (X=50, Y=360)                      Center @ (X=510, Y=360)
      ┌───────────┬───────────┐                   ┌───────────┬───────────┐
 +Y   │ Red (R)   │ Green (G) │              +Y   │ Red (R)   │ Green (G) │
  ▲   ├───────────┼───────────┤               ▲   ├───────────┼───────────┤
  │   │ Blue (B)  │ Yellow (Y)│               │   │ Blue (B)  │ Yellow (Y)│
      └───────────┴───────────┘                   └───────────┴───────────┘
```

| Tray / Compartment |         World Coordinates $(X, Y, Z)$         | Target Color | Drop Method           |
| ------------------ | :-------------------------------------------: | :----------: | --------------------- |
| **Tray 1 — Red**   | $(30.0\text{ mm}, 380.0\text{ mm}, 35.0\text{ mm})$ |     Red      | Top-down gravity drop |
| **Tray 1 — Green** | $(70.0\text{ mm}, 380.0\text{ mm}, 35.0\text{ mm})$ |    Green     | Top-down gravity drop |
| **Tray 1 — Blue**  | $(30.0\text{ mm}, 340.0\text{ mm}, 35.0\text{ mm})$ |     Blue     | Top-down gravity drop |
| **Tray 1 — Yellow**| $(70.0\text{ mm}, 340.0\text{ mm}, 35.0\text{ mm})$ |    Yellow    | Top-down gravity drop |
| **Tray 2 — Red**   | $(490.0\text{ mm}, 380.0\text{ mm}, 35.0\text{ mm})$|     Red      | Top-down gravity drop |
| **Tray 2 — Green** | $(530.0\text{ mm}, 380.0\text{ mm}, 35.0\text{ mm})$|    Green     | Top-down gravity drop |
| **Tray 2 — Blue**  | $(490.0\text{ mm}, 340.0\text{ mm}, 35.0\text{ mm})$|     Blue     | Top-down gravity drop |
| **Tray 2 — Yellow**| $(530.0\text{ mm}, 340.0\text{ mm}, 35.0\text{ mm})$|    Yellow    | Top-down gravity drop |
