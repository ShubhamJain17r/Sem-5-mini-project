# Single xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

A single xArm 1s robotic arm picks randomly scattered colored wooden cubes from a **flat rectangular arena ($260\times 140\text{ mm}$, $Z = 0.0\text{ mm}$)** and sorts them into a **single dedicated 3D-printed 2×2 Collection Tray** mounted on its immediate left flank.

The operator views the setup from the **FRONT**. The entire workcell is constructed on a single, store-cut **flat wooden baseboard** ($400\text{ mm} \text{ wide} \times 400\text{ mm} \text{ deep} \times 18\text{ mm}$ MDF). The arm is mounted flat at $(X = 200.0\text{ mm}, Y = 280.0\text{ mm}, Z = 0.0\text{ mm})$, with Motor 6 oriented at nominal heading $-90.0^\circ$ (South) directly into the picking arena:

- **Picking Arena (Flat Tabletop Geometry)**: Sits directly flat on the wooden baseboard at $Z = 0.0\text{ mm}$ ($260\text{ mm}$ wide $\times 140\text{ mm}$ depth, $X \in [70.0, 330.0\text{ mm}]$, $Y \in [40.0, 180.0\text{ mm}]$). Retained by a soft compliant **$10\text{ mm}$ self-adhesive foam tape perimeter** to prevent cubes from rolling off. The single arm completely covers this active arena ($r \in [100.0, 272.9\text{ mm}]$) with zero blind spots.
- **Drop Zone (Immediate Left Flank 2×2 Tray)**:
  - **Single 2×2 Tray**: Dedicated to `xarm_1`, centered at $(X = 50.0\text{ mm}, Y = 280.0\text{ mm}, Z = 0.0\text{ mm})$ with 4 color compartments (Red, Green, Blue, Yellow).
  - **Drop Cycle Optimization**: Positioned on the left flank ($r = 150.0\text{ mm}$), the drop requires a clean **$90^\circ$ clockwise swing** from the arena heading (drop yaw tick $\approx 145$), minimizing cycle time and eliminating servo stall.
- **Rear Electronics & Power Bay**: The area behind the arm base ($X \in [140, 260\text{ mm}], Y \in [330, 390\text{ mm}]$) hosts the NVIDIA Jetson Orin Nano / Raspberry Pi 5 and single 7.4V / 5A buck regulator. Surface adhesive cable clips route USB serial and 7.4V power cleanly to the arm.
- **Kinematic Sweet Spot**: The arm is mounted flat at $Z = 0\text{ mm}$, utilizing the built-in $65\text{ mm}$ height to Motor 5. Pick radius is bounded between $R_{\min} = 100\text{ mm}$ and $R_{\max} = 280\text{ mm}$, maintaining healthy elbow pitch angles to eliminate joint singularities and motor stall.
- **Coplanar Board-Mounted ArUco Fiducials**: 4 ArUco markers (IDs 0–3, $30\times 30\text{ mm}$) printed on matte vinyl are applied flat to the wooden board at the four arena corners at $Z = 0.0\text{ mm}$, ensuring zero parallax error during planar homography ($520\times 280\text{ px}$ grid at $2.0\text{ px/mm}$).

An overhead USB camera mounted on a rear boom stand at $H \approx 650\text{ mm}$ captures the picking arena nadir over $(200, 110\text{ mm})$. The system runs under **ROS 2 Humble** on a NVIDIA Jetson Orin Nano / Raspberry Pi 5.

> [!NOTE]
> For in-depth mechanical, kinematic, optical, and diagnostic specifications, consult the dedicated stepping stone documents:
>
> - **Stepping Stone 0**: [`docs/00_docker_and_ros2_development_guide.md`](00_docker_and_ros2_development_guide.md) — Docker containerization, hybrid host/container workflow, and ROS 2 Humble package development standards.
> - **Stepping Stone 1**: [`docs/01_arena_and_layout.md`](01_arena_and_layout.md) — $400\times 400\text{ mm}$ baseboard, single arm mount at $(200, 280)$, Left Flank tray at $(50, 280)$, and $260\times 140\text{ mm}$ arena.
> - **Stepping Stone 2**: [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) — Analytical closed-form IK ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque/friction mechanics, and 5-phase pick trajectory.
> - **Stepping Stone 3**: [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md) — Camera mounting optics, 4-point ArUco planar homography, hardware exposure lock, HSV color segmentation, and spatial centroid extraction.
> - **Stepping Stone 4**: [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md) — Distributed ROS 2 Humble node graph, custom interfaces, single-arm scheduling pipeline, and production Python nodes.
> - **Stepping Stone 5**: [`docs/05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md) — Low-level serial packet protocol, Arduino Uno R4 WiFi test bench, servo deviation zeroing, and tick-to-angle mapping.
> - **Stepping Stone 6**: [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md) — Nearest-First corridor collision avoidance, Motor 1 grip verification, empirical Z-sag compensation, and lens undistortion.

---

## Hardware Specifications

| Component        | Spec                                                                                                                                                |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arm              | 1× xArm 1s mounted at $(X = 200.0\text{ mm}, Y = 280.0\text{ mm})$, heading $-90.0^\circ$ (South)                                                  |
| Baseboard        | Single store-cut MDF board ($400\text{ mm} \times 400\text{ mm} \times 18\text{ mm}$, zero woodworking cuts, ~2.3 kg)                              |
| Compute          | NVIDIA Jetson Orin Nano / Raspberry Pi 5 (Rear Bay $Y \in [330, 390\text{ mm}]$, adhesive standoffs)                                                |
| Camera           | Logitech C270 or C920 (USB) mounted on rear welded aluminum stand at $H \approx 650\text{ mm}$ nadir over $(200, 110\text{ mm})$                    |
| Picking Arena    | Flat tabletop deck ($Z = 0.0\text{ mm}$, $260\times 140\text{ mm}$) with $10\text{ mm}$ foam retaining tape perimeter                               |
| ArUco Setup      | 4 coplanar markers (IDs 0–3, $30\times 30\text{ mm}$) applied flat at $(70, 180), (330, 180), (70, 40), (330, 40)$ at $Z = 0.0\text{ mm}$          |
| Arena Limits     | $X \in [70, 330\text{ mm}]$, $Y \in [40, 180\text{ mm}]$, $R \in [100, 273\text{ mm}]$ (100% reachable within $R_{\max} = 280\text{ mm}$)           |
| Collection Box   | 1× 3D-printed 2x2 flank tray ($80\times 80\times 35\text{ mm}$, 4 color slots) at Left Flank $(50, 280)$                                            |
| Electronics Bay  | Rear Bay ($X \in [140, 260\text{ mm}], Y \in [330, 390\text{ mm}]$), single 7.4V / 5A buck regulator, surface adhesive cable clips                  |
| Objects          | Wooden craft cubes (25 mm, $15\text{ g}$), spray-painted in 4 high-contrast matte colors                                                             |
| Colors           | Red, Green, Blue, Yellow (HSV-separated)                                                                                                            |
| Connectivity     | USB-Serial (`/dev/ttyUSB0`) to ROS 2 driver via direct surface wiring                                                                               |

---

## Field Layout

```text
  Y (Depth: 400 mm)
   ^
   │  ┌────────────────────────────────────────────────────────┐
   │  │ REAR OF WORKSPACE (Y = 400 mm)                         │
   │  │          [Clamp-On Overhead Stand Base]                │
   │  │                  (X = 200, Y = 390)                    │
   │  │                                                        │
   │  │          ┌──────────────────────────────────┐          │
   │  │          │    REAR ELECTRONICS & PSU BAY    │          │
   │  │          │ (RPi 5 / Jetson & 7.4V Regulator)│          │
   │  │          └────────────────┬─────────────────┘          │
   │  │                           │                            │
   │  │   ┌─────────────┐       xArm 1                         │
   │  │   │  2x2 Tray   │     (Base Z = 0)                     │
   │  │   │(R|G|B|Y Bins│    Heading: -90°                     │
   │  │   │  (50, 280)  │      (200, 280)                      │
   │  │   └─────────────┘          │                           │
   │  │                            ▼                           │
   │  │     [ArUco #0] ────────────┼──────────── [ArUco #1]    │
   │  │     (70, 180)              │             (330, 180)    │
   │  │                            │                           │
   │  │                   ACTIVE PICKING ARENA                 │
   │  │                         (Z = 0)                        │
   │  │                       260 x 140 mm                     │
   │  │                            │                           │
   │  │     [ArUco #2] ────────────┴──────────── [ArUco #3]    │
   │  │     (70, 40)   (Foam Tape Border Barrier)(330, 40)     │
   │  │                                                        │
   │  │ FRONT OF WORKSPACE (Operator Position @ Y = 0 mm)      │
   │  └────────────────────────────────────────────────────────┘
   └───────────────────────────────────────────────────────────────> X (Width: 400 mm)
```

---

## Kinematics & Elevation Optimization

1. **Shoulder Height Leverage**: The arm's Motor 5 (shoulder pitch axis) is **$65.0\text{ mm}$** above the baseboard surface.
2. **Flat Baseboard Arena ($Z = 0.0\text{ mm}$)**: The picking arena floor sits flat on the wooden baseboard, placing the cube center at **$Z \approx 12.5\text{ mm}$**. The vertical drop from Motor 5 is $\approx 52.5\text{ mm}$. The arm approaches horizontally or at shallow pitch in its strong mechanical torque band.
3. **Radial Boundary ($R_{\max} = 280.0\text{ mm}$)**: Bounding maximum reach at $280\text{ mm}$ backs off safely before mechanical lockup ($344\text{ mm}$), maintaining a healthy elbow angle to prevent motor stalling.
4. **Strict Coplanar Marker Alignment**: Mounting all 4 ArUco markers flat on the baseboard ($Z = 0.0\text{ mm}$) guarantees zero elevation perspective distortion during planar homography calculation.
5. **Immediate Left Flank Drop Swing ($90.0^\circ$)**: Placing the 2×2 grid sorting tray on the left flank at $(50, 280)$ ($r = 150\text{ mm}$) enables a clean $90^\circ$ clockwise swing from arena heading (drop yaw tick $\approx 145$), minimizing cycle time and eliminating servo stall.
6. **Full Analytical Inverse Kinematics (IK) & Grasp Planning**: See the dedicated mathematical and mechanical architecture document [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md) for closed-form joint angle equations ($\theta_1\text{--}\theta_5$), dual-regime roll control, torque models, and 5-phase trajectory waypoint generation.
7. **Nearest-First Pick Policy (Corridor Collision Avoidance)**: To prevent horizontal claw approaches from plowing through foreground cubes, the queue is strictly sorted by ascending radial distance $r$. See [`docs/06_system_testing_diagnostics_and_verification.md`](06_system_testing_diagnostics_and_verification.md).
8. **Empirical Z-Sag Compensation**: To counteract mechanical gear backlash and linkage flex at reach ($r > 180\text{ mm}$), commanded elevation incorporates a linear sag offset $Z_{\text{cmd}} = Z_{\text{target}} + k_{\text{sag}} \cdot (r - 180)$.
9. **Software Grip Confirmation via Motor 1 (Claw / $\theta_6$) Readback**: Post-clamping position readback ($< 220\text{ ticks} = \text{miss}$, $380\text{--}520\text{ ticks} = \text{cube grasped}$) prevents empty drop swings.

> [!NOTE]
> **Kinematic Joint Indices vs. Physical Hardware Motor IDs**:
> In mathematical kinematics ($\theta_1\text{--}\theta_6$), joints are indexed base-to-tip ($\theta_1 = \text{Base Yaw}, \theta_2 = \text{Shoulder Pitch}, \dots, \theta_6 = \text{Claw Clamp}$).
> On the physical **Hiwonder xArm 1s** bus servo daisy chain, hardware IDs are addressed tip-to-base: Hardware ID 1 = Claw Gripper, ID 2 = Wrist Roll, ID 3 = Wrist Pitch, ID 4 = Elbow Pitch, ID 5 = Shoulder Pitch, ID 6 = Base Yaw. Both `config/xarm_hardware.yaml` and the ROS 2 driver explicitly preserve this mapping.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```text
Frame from camera (Overhead @ 65 cm above arena, nadir @ X=200, Y=110)
  → Intrinsic Lens Undistortion: cv2.undistort() with camera matrix K and distortion vector D
  → Detect 4 ArUco markers framing the flat arena (Z = 0.0 mm):
      - ArUco #0: Rear-Left   (X = 70.0 mm, Y = 180.0 mm)
      - ArUco #1: Rear-Right  (X = 330.0 mm, Y = 180.0 mm)
      - ArUco #2: Front-Left  (X = 70.0 mm, Y = 40.0 mm)
      - ArUco #3: Front-Right (X = 330.0 mm, Y = 40.0 mm)
  → Compute & freeze perspective transform → rectified top-down view (520 x 280 px @ 2.0 px/mm)
  → Convert rectified image to HSV
  → Threshold for 4 colors (Red, Green, Blue, Yellow) → binary masks
  → Find contours per mask → filter by area (25 mm cube profile)
  → Compute centroid (pixel) per detected cube
  → Map to arena mm: (X_arena, Y_arena)
  → Convert to Arm Polar Frame relative to arm base (200, 280):
      ΔX = X_arena - 200.0,   ΔY = Y_arena - 280.0
      r = sqrt(ΔX² + ΔY²),    θ = atan2(ΔY, ΔX) - (-90.0°)
  → Boundary check: Verify 100 mm ≤ r ≤ 280 mm within arm operating sector
  → Nearest-First Sort: Order queue by ascending r
  → Output: [{color, x_mm, y_mm, r_mm, theta_deg, target_arm: "xarm_1"}, ...]
```

---

## ROS 2 Architecture & Coordination

_(Full mathematical derivations, QoS profiles, launch scripts, and Python implementations detailed in [`docs/04_ros2_architecture_and_scheduling.md`](04_ros2_architecture_and_scheduling.md))_:

### Nodes

| Node                     | Role                                                                                                                                 |             Rate              |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------ | :---------------------------: |
| `camera_node`            | Acquires frames from `/dev/video0` with exposure lock ($156$) and WB lock ($4500\text{K}$) → `/camera/image_raw`                     |        $30\text{ Hz}$         |
| `vision_node`            | Executes 4-point ArUco homography warp + HSV color segmentation → `/vision/detected_objects`                                         |   $10\text{--}15\text{ Hz}$   |
| `autonomous_sorter_node` | Nearest-First sorting queue management, goal dispatch, and trajectory sequencing                                                    |      $20\text{ Hz}$ loop      |
| `driver_node`            | Manipulator controller: closed-form analytical IK, 5-phase trajectory profiling, serial packet writer (`/dev/ttyUSB0`)                | Event + $50\text{ Hz}$ status |
| `dashboard_node`         | System telemetry, cycle times, sorting counts, and diagnostics via Web UI                                                            |         $5\text{ Hz}$         |

### Custom Message Interfaces

```text
DetectedObject.msg      → uint32 object_id, string color, float64 x_mm, float64 y_mm, float64 z_mm, float64 yaw_deg, float64 confidence
DetectedObjectArray.msg  → std_msgs/Header header, DetectedObject[] objects, uint32 total_detected, bool homography_locked
ArmTask.msg             → uint32 task_id, string arm_id, DetectedObject target_object, float64 target_r_mm, float64 target_theta_deg, string destination_box, int32 compartment_index, float64 drop_theta_deg
ArmStatus.msg           → std_msgs/Header header, string arm_id, string state, uint32 active_task_id, bool is_busy, bool has_cube, float64[5] joint_angles_deg, float64 bus_voltage_v, string error_code
```

---

## Arm Motion Sequence (per task)

### Distance-Based Pitch Lookup Table (LUT)

_(Full closed-form equations and empirical calibration detailed in [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md))_:

- **Near** ($r \in [100, 150\text{ mm}]$): Steep downward approach ($\phi \in [-75^\circ, -55^\circ]$, Object-Aligned Roll $\theta_5$)
- **Mid** ($r \in [150, 220\text{ mm}]$): Angled approach ($\phi \in [-45^\circ, -25^\circ]$, Object-Aligned Roll $\theta_5$)
- **Far Boundary** ($r \in [220, 280\text{ mm}]$): Shallow horizontal approach ($\phi \in [-20^\circ, -5^\circ]$, Locked Level Roll $\theta_5 = 0^\circ$)

### Motion Execution Steps

```text
1. rotate_base(theta)            → rotate arm base to target azimuth into arena
2. lookup_pitch(r)               → query pitch angle α from LUT for radius r (or analytical IK)
3. set_roll(0°)                  → lock wrist roll level with table
4. move_standoff(r - 30mm, z)    → position gripper backed off radially along approach vector
5. open_gripper(55mm)            → fully open 2-finger claw
6. advance_radially(r, pick_z)   → slide forward onto flat arena surface (Z ≈ 12.5 mm)
7. close_gripper()               → grip cube (mechanical self-centering against friction mat)
8. check_grip()                  → read servo position feedback; retry if failed
9. ascend_and_retract()          → lift slightly and retract radially
10. retract_to_compact()         → tuck arm into compact travel pose
11. rotate_to_drop_box()         → swing 90° clockwise to Left Flank Tray (drop yaw tick ≈ 145)
12. extend_to_bin(bin_x, bin_y)  → position over target color cell
13. descend(bin_z)               → lower into cell (Z ≈ 40 mm)
14. open_gripper()               → release cube
15. ascend()                     → clear box rim
16. return_home()                → ready pose
```

---

## Collection Box Coordinates (`config/xarm_hardware.yaml`)

Single 3D-printed 2×2 sorting tray ($80\times 80\times 35\text{ mm}$) located on the left flank of the arm:

```text
                Tray 1 (Left Flank: xArm 1 @ 200, 280)
                        Center @ (X=50, Y=280)
                     ┌───────────┬───────────┐
                +Y   │ Red (0)   │ Green (1) │
                 ▲   ├───────────┼───────────┤
                 │   │ Blue (2)  │ Yellow (3)│
                     └───────────┴───────────┘
```

| Compartment | World Coordinates $(X, Y, Z)$ | Target Color | Drop Method |
| :--- | :---: | :---: | :--- |
| **Slot 0 — Red** | $(30.0\text{ mm}, 300.0\text{ mm}, 35.0\text{ mm})$ | Red | Top-down gravity drop |
| **Slot 1 — Green** | $(70.0\text{ mm}, 300.0\text{ mm}, 35.0\text{ mm})$ | Green | Top-down gravity drop |
| **Slot 2 — Blue** | $(30.0\text{ mm}, 260.0\text{ mm}, 35.0\text{ mm})$ | Blue | Top-down gravity drop |
| **Slot 3 — Yellow** | $(70.0\text{ mm}, 260.0\text{ mm}, 35.0\text{ mm})$ | Yellow | Top-down gravity drop |
