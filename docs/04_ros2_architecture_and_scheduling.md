# Stepping Stone 4: ROS 2 Architecture & Motion Control — Single xArm 1S

This document serves as the definitive engineering specification for **Stepping Stone 4** in the single-arm automated color-sorting robot system. It details the distributed software architecture running under **ROS 2 Humble Hawksbill** on the **NVIDIA Jetson Orin Nano / Orin NX (or Raspberry Pi 5)**.

It covers the complete computation graph, message interfaces, Nearest-First queue prioritization, analytical closed-form inverse kinematics, 5-phase minimum-jerk trajectory execution, production Python node implementations, and launch configurations.

---

## 1. System Overview & Computation Graph Topology

### 1.1 Architectural Philosophy

The single-arm sorting system is structured as a **loosely coupled, asynchronous, event-driven ROS 2 computation graph**:

1. **Decoupled Perception and Action**: The computer vision pipeline (`vision_node`) operates strictly as an observational telemetry producer, detecting physical cube positions in millimeter arena coordinates without coupling to arm state.
2. **Nearest-First Autonomous Sorter (`autonomous_sorter_node`)**: Aggregates object detections, prioritizes cubes in ascending radius (nearest-first) to prevent arm-to-cube foreground collisions, computes analytical IK, and coordinates the 5-phase pick and drop sequence.
3. **Hardware Driver Node (`driver_node`)**: Manages low-level binary serial protocol communication with the xArm 1S controller over `/dev/ttyUSB0`, dispatches multi-servo moves, tracks potentiometer feedback, and enforces emergency stop safety.
4. **Lightweight Analytical Execution vs. Heavy MoveIt**: Rather than relying on heavy numerical trajectory frameworks (e.g., MoveIt 2), motion is governed by closed-form geometric IK running in $< 0.1\text{ ms}$ per waypoint with zero iterative convergence issues.
5. **Elimination of Dual-Arm Contention**: With a single manipulator, inter-arm collision avoidance, territorial partitioning, and spatial mutexes are eliminated, resulting in a streamlined, highly reliable sorting pipeline.

### 1.2 ROS 2 Computation Graph

```mermaid
flowchart TD
    subgraph Hardware ["Physical Hardware Layer"]
        CAM["Overhead USB Webcam<br/>(Logitech C270/C920 @ 65–70cm)"]
        USB1["Hiwonder xArm 1S Controller<br/>(/dev/ttyUSB0 @ 115200)"]
    end

    subgraph Perception ["Perception Pipeline"]
        CN["camera_node<br/>(30 Hz V4L2 Capture)"]
        VN["vision_node<br/>(10–15 Hz ArUco Warp + HSV)"]
    end

    subgraph Execution ["Autonomous Execution Layer"]
        SN["autonomous_sorter_node<br/>(Nearest-First Queue + IK State Machine)"]
        DN["xarm_driver_node<br/>(Serial Protocol Driver)"]
    end

    subgraph Monitoring ["Diagnostics & Web Config"]
        DASH["web_config / Port 8000<br/>(Live Telemetry & Diagnostics)"]
    end

    CAM -->|V4L2 Frame| CN
    CN -->|/camera/image_raw (sensor_msgs/Image)| VN
    VN -->|/vision/detected_objects (DetectedObjectArray)| SN

    SN -->|/xarm_1/joint_commands (JointState)| DN
    DN -->|115200 Baud Binary Packets| USB1

    VN -->|/vision/annotated_image| DASH
    DN -->|/xarm_1/joint_states| DASH
```

### 1.3 Node Specifications & Topic Registry

| Node Name                   | Input Topics / Interfaces                 | Output Topics / Interfaces                                              |            Execution Rate            |            QoS Profile             | Role                                                                                                                                                 |
| --------------------------- | ----------------------------------------- | ----------------------------------------------------------------------- | :----------------------------------: | :--------------------------------: | ---------------------------------------------------------------------------------------------------------------------------------------------------- |
| **`camera_node`**           | Hardware `/dev/video0`                    | `/camera/image_raw`  `/camera/camera_info`                               |            $30\text{ Hz}$            | SensorData (Best Effort, Depth 1)  | V4L2 video acquisition, exposure lock ($156$), white balance lock ($4500\text{K}$).                                                                  |
| **`vision_node`**           | `/camera/image_raw`                       | `/vision/detected_objects`  `/vision/annotated_image`                    |      $10\text{--}15\text{ Hz}$       | Reliable, Transient Local, Depth 1 | ArUco 4-point homography warp ($2.0\text{ px/mm}$), HSV color segmentation, centroid extraction $(X_w, Y_w)$, yaw $\psi \in [-45^\circ, +45^\circ]$. |
| **`autonomous_sorter_node`**| `/vision/detected_objects`                | `/xarm_1/joint_commands`                                                |         $10\text{ Hz}$ loop          |         Reliable, Depth 10         | Nearest-First target ordering, closed-form analytical IK for pick & Left Flank drop, 5-phase trajectory execution.                                    |
| **`driver_node`**           | `/xarm_1/joint_commands`                  | `/xarm_1/joint_states`  `/dev/ttyUSB0` (UART)                           | Event-driven + $50\text{ Hz}$ status |         Reliable, Depth 5          | xArm 1S serial protocol driver: command dispatch, position telemetry feedback, health monitoring & E-Stop interlock.                                 |
| **`web_config`**            | HTTP / REST API                           | Port 8000 Web Dashboard                                                 |            $2\text{ Hz}$             |            Best Effort             | Real-time calibration, servo min/max adjustments, live potentiometer readouts, and CSV drift telemetry logging.                                      |

---

## 2. Custom ROS 2 Interface Definitions

All custom messages and services are defined in the project ROS 2 package `xarm_interfaces`.

### 2.1 `DetectedObject.msg`

Represents an individual physical cube identified on the elevated picking arena.

```text
# xarm_interfaces/msg/DetectedObject.msg
# Represents a single detected colored wooden block on the picking arena

uint32 object_id                  # Unique sequential detection ID
string color                      # Color classification: "RED", "GREEN", "BLUE", "YELLOW"
float64 x_mm                      # Absolute world X coordinate in arena frame [80.0, 480.0 mm]
float64 y_mm                      # Absolute world Y coordinate in arena frame [40.0, 420.0 mm]
float64 z_mm                      # Elevation coordinate (nominally +37.5 mm for cube centroid on +25mm deck)
float64 yaw_deg                   # Planar orientation angle psi in range [-45.0, +45.0 degrees]
float64 confidence                # Optical classification confidence score [0.0, 1.0]
builtin_interfaces/Time stamp     # Timestamp of perception capture
```

### 2.2 `DetectedObjectArray.msg`

Perception telemetry snapshot published by `vision_node`.

```text
# xarm_interfaces/msg/DetectedObjectArray.msg
# Complete perceived state of the picking arena at a given instant

std_msgs/Header header            # Sequence counter and frame_id ("arena_frame")
DetectedObject[] objects          # Array of currently visible candidate cubes
uint32 total_detected             # Count of active objects
bool homography_locked            # True if all 4 ArUco markers are actively tracked
```

### 2.3 `ArmTask.msg`

Dispatched by `scheduler_node` to command a specific pick-and-place cycle.

```text
# xarm_interfaces/msg/ArmTask.msg
# Action directive issued from scheduler to an individual arm controller

uint32 task_id                    # Unique task transaction ID
string arm_id                     # Target manipulator: "arm_1" or "arm_2"
DetectedObject target_object      # Full perception state of the cube to pick

# Kinematic target parameters relative to target arm base
float64 target_r_mm               # Polar radius r = sqrt(dx^2 + dy^2) [120.0, 320.0 mm]
float64 target_theta_deg          # Polar azimuth angle theta = atan2(dy, dx) [degrees]
float64 distance_from_edge_cm     # Physical reach d_edge = (r - 70 mm) / 10 [5.0, 25.0 cm]

# Sorting destination parameters
string destination_box            # Drop target: "BOX_1_NW" (Arm 1) or "BOX_2_SW" (Arm 2)
int32 compartment_index           # Compartment index 0..3 matching color partition
float64 drop_theta_deg            # Target base yaw for drop (+125° for NW, -125° for SW)

# Safety synchronization
bool requires_mutex               # True if target cube lies within central crossover band
```

### 2.4 `ArmStatus.msg`

High-speed state telemetry published by each `arm_node` at $50\text{ Hz}$.

```text
# xarm_interfaces/msg/ArmStatus.msg
# Runtime status and telemetry of an individual xArm 1S

std_msgs/Header header
string arm_id                     # "arm_1" (North) or "arm_2" (South)

# State Machine State
string state                      # "IDLE", "APPROACHING", "GRASPING", "LIFTING",
                                  # "TRANSIT_DROP", "RELEASING", "RETRACTING", "FAULT"
uint32 active_task_id             # ID of task currently in execution (0 if IDLE)
bool is_busy                      # True if executing a trajectory phase
bool has_cube                     # True if gripper confirms active clamp on block
bool holds_mutex                  # True if arm currently holds the central crossover lock

# Current Joint Angles (degrees)
float64 motor_1_azimuth_deg
float64 motor_2_shoulder_deg
float64 motor_3_elbow_deg
float64 motor_4_wrist_pitch_deg
float64 motor_5_wrist_roll_deg
float64 motor_6_gripper_pwm

# Diagnostics
float64 bus_voltage_v             # Real-time voltage reported by servo telemetry (nominal 7.4V)
string error_code                 # "NONE", "IK_UNREACHABLE", "SERVO_TIMEOUT", "MUTEX_TIMEOUT"
```

### 2.5 Autonomous Sorting Control Services

```text
# std_srvs/srv/Trigger.srv
---
bool success                      # True if sorting sequence initiated / stopped
string message                    # Status / diagnostic confirmation
```

The `autonomous_sorter_node` provides standard ROS 2 service endpoints:
- `/autonomous_sorter/start`: Initiates perception scanning and autonomous pick-and-place execution.
- `/autonomous_sorter/stop`: Safely aborts execution at the end of the current motion phase and returns the arm to `ready_standoff`.## 3. Nearest-First Radial Queue Scheduling & Collision-Free Planning

### 3.1 Spatial Geometry & Workcell Layout

The flat rectangular arena workspace measures $260.0\text{ mm} \times 140.0\text{ mm}$ on the table surface ($Z = 0.0\text{ mm}$) with world coordinates $X \in [70.0, 330.0\text{ mm}]$ and $Y \in [40.0, 180.0\text{ mm}]$.

- **Central Manipulator (`xarm_1`)**: Base center located at $(X_0 = 200.0\text{ mm}, Y_0 = 280.0\text{ mm}, Z_0 = 0.0\text{ mm})$, with nominal heading $-90.0^\circ$ (pointing directly South, $-Y$, into the picking arena).
- **Left Flank 2×2 Sorting Tray**: Located at $(X_t = 50.0\text{ mm}, Y_t = 280.0\text{ mm}, Z_t = 0.0\text{ mm})$, reached by an outward $90^\circ$ clockwise swing from the picking heading ($r = 150.0\text{ mm}$, base yaw tick $\approx 145$).
- **Collision-Free Clearance**: The arena rear boundary at $Y = 180\text{ mm}$ provides $+23\text{ mm}$ of clear physical margin before the robot metal baseplate edge ($Y = 203\text{ mm}$).

```text
  Y (mm)
   280 ┌───────────────────┐               [xArm 1S Base]
       │ Left Flank 2x2    │               (200, 280, 0)
       │ Sorting Tray      │                     │
       │ (50, 280)         │                     │ +23 mm Clearance
   180 └───────────────────┘ ════════════════════╪═══════════════════════
       │ [ArUco #0] (70, 180)                    │  [ArUco #1] (330, 180)
       │                                         │
       │                 ACTIVE PICKING ARENA    │
       │                (Flat Z = 0.0 mm Plane)  ▼ (Heading -90° South)
       │                                         
       │ [ArUco #2] (70, 40)                     │  [ArUco #3] (330, 40)
    40 └─────────────────────────────────────────┴───────────────────────
       70                                                              330 ───> X (mm)
```

### 3.2 Nearest-First Radial Queue Policy

To prevent the manipulator from knocking over foreground cubes while reaching for background objects, the scheduler applies a **Nearest-First radial ordering policy**:

1. **Polar Distance Metric**: For each detected cube at $(X_w, Y_w)$, its radial distance $r$ from the arm base is calculated:
   $$r = \sqrt{(X_w - X_0)^2 + (Y_w - Y_0)^2}$$
2. **Ascending Radial Sort**: All candidate pick targets are sorted in ascending order of $r$:
   $$\mathcal{Q} = [c_1, c_2, \dots, c_n] \quad \text{such that } r(c_1) \le r(c_2) \le \dots \le r(c_n)$$
3. **Plowing Prevention**: Because $Y_0 = 280\text{ mm}$, objects closer to the arm base have *larger* $Y$ coordinates and *smaller* radial distances $r$. By picking these foreground items first, the arm works from near to far, guaranteeing an unobstructed corridor for subsequently sorted objects.

### 3.3 Kinematic Reach & Boundary Guardrails

Before queuing any candidate cube, the scheduler verifies that the polar coordinates satisfy the physical reachable envelope:
$$r \in [130.0\text{ mm}, 265.0\text{ mm}], \quad \theta_{\text{arm}} \in [-35.0^\circ, +35.0^\circ]$$
Targets outside these limits are flagged with `IK_UNREACHABLE` and skipped.

---

## 4. Single-Arm 5-Phase Minimum-Jerk Trajectory Lifecycle

Each sorting transaction executes 5 distinct kinematic phases:

### 4.1 Trajectory Phase Breakdown

|    Phase    | Description                          |          Altitude $Z$           |             Radial Span $r$              |       Duration / Speed       | Action Details |
| :---------: | ------------------------------------ | :-----------------------------: | :--------------------------------------: | :--------------------------: | :------------- |
| **Phase 1** | **Azimuth Slew & Standoff Approach** |     Hover ($+40.0\text{ mm}$)   | Retracted $\rightarrow r - 30\text{ mm}$ |    Fast ($500\text{ ms}$)    | Slew base yaw to target $\theta$, pre-open claw to 200 ticks |
| **Phase 2** | **Horizontal Radial Advance**        |   Descent ($+12.5\text{ mm}$)   |     $r - 30\text{ mm} \rightarrow r$     |   Smooth ($450\text{ ms}$)   | Advance claw tip around cube; orient wrist roll to match cube yaw $\psi$ |
| **Phase 3** | **Gripper Clamping & Settling**      |  Clamping ($+12.5\text{ mm}$)   |              At target $r$               | Grip Pulse ($250\text{ ms}$) | Clamp claw (tick 500); settle against silicone pad |
| **Phase 4** | **Vertical Lift & Radial Retract**   |     Lift to $+60.0\text{ mm}$   |     $r \rightarrow r - 40\text{ mm}$     |   Smooth ($400\text{ ms}$)   | Lift cube clear of perimeter tape; retract toward shoulder |
| **Phase 5** | **Flank Swivel & Release**           | Drop altitude ($+45.0\text{ mm}$) |   Swivel to Left Flank Tray (90° CW)   |    Fast ($600\text{ ms}$)    | Swing to tray compartment, open claw (tick 200), return to ready standoff |

### 4.2 Cycle Timing & Performance

- **Single Cube Cycle Time**: Complete pick-and-drop requires $\approx 2.2\text{--}2.5\text{ seconds}$.
- **System Throughput**: Steady-state sorting rate of **$24\text{--}26\text{ cubes/minute}$**.
- **Jerk Reduction**: Quintic polynomial scaling $s(u) = 10u^3 - 15u^4 + 6u^5$ enforces zero initial and final velocity and acceleration ($\dot{s}(0) = \dot{s}(1) = 0, \ddot{s}(0) = \ddot{s}(1) = 0$), eliminating structural vibration and servo gear backlash.

---

## 5. Production ROS 2 Node Architecture

The single xArm sorting system is implemented across three modular production nodes in the `sorting_arm` package:

### 5.1 `AutonomousSorterNode` (`ros2_ws/src/sorting_arm/sorting_arm/autonomous_sorter_node.py`)

The master autonomous sorting coordinator orchestrates the entire perception-to-motion loop:

```python
class AutonomousSorterNode(Node):
    """ROS 2 Node orchestrating vision-guided pick-and-place color sorting."""

    def __init__(self):
        super().__init__("autonomous_sorter_node")

        # Declare parameters
        self.declare_parameter("arm_id", "xarm_1")
        self.declare_parameter("config_dir", "config")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("dry_run", False)

        # Subscriptions & Publishers
        self.create_subscription(String, "/vision/detected_objects", self.objects_callback, 10)
        self.create_subscription(JointState, f"/{self.arm_id}/joint_states", self.joint_states_callback, 10)
        self.joint_cmd_pub = self.create_publisher(JointState, f"/{self.arm_id}/joint_commands", 10)

        # Control services
        self.create_service(Trigger, "/autonomous_sorter/start", self.start_callback)
        self.create_service(Trigger, "/autonomous_sorter/stop", self.stop_callback)
```

#### Core Operational Loop
1. **Perception Subscription**: Subscribes to `/vision/detected_objects` published by `vision_node`.
2. **Nearest-First Ordering**: Filters detected cubes and sorts the active queue in ascending order of radial distance $r$.
3. **Analytical Trajectory Generation**:
   - For pick: Solves `solve_ik(r, theta, z=12.5, roll_tick)` with roll aligned to cube orientation $\psi$.
   - For drop: Retrieves compartment target from `get_compartment_target(color, arm_id)` targeting the Left Flank 2×2 tray ($90^\circ$ CW swing, drop yaw tick $\approx 145$).
4. **Trajectory Execution**: Emits interpolated `sensor_msgs/JointState` waypoints to `/{arm_id}/joint_commands`.


### 5.2 `DriverNode` (`ros2_ws/src/sorting_arm/sorting_arm/driver_node.py`)

The hardware interface node connects directly to the LewanSoul xArm controller:

- **Automatic Port Discovery**: Scans `/dev/xarm_*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, and native USB-HID devices to automatically detect and bind the physical xArm 1S controller.
- **Binary Serial Protocol**: Assembles and sends 6-byte binary motion command frames (`0x55 0x55 ...`) to the LX-15D bus servo controller.
- **Closed-Loop Position Feedback**: Continuously queries servo potentiometers and publishes real-time joint positions on `/{arm_id}/joint_states`.
- **Dynamic Safety Guardrails**: Enforces dynamic deviation clamping (maximum $\pm 25\text{ ticks}$) to prevent hardware damage, alongside emergency stop service handling (`/{arm_id}/emergency_stop`).

### 5.3 `VisionNode` (`ros2_ws/src/sorting_arm/sorting_arm/vision_node.py`)

The perception node interfaces with OpenCV:
- Streams overhead video from `/dev/video0`.
- Detects the 4 ArUco markers (`DICT_4X4_50`, IDs 0..3) coplanar with the arena tabletop ($Z = 0.0\text{ mm}$).
- Computes perspective homography matrix $H$ and warps the arena to an exact $2.0\text{ px/mm}$ metric grid ($520 \times 280\text{ px}$).
- Segments Red, Green, Blue, and Yellow cubes via calibrated HSV thresholds.
- Computes spatial moments to extract cube centroids $(X_w, Y_w)$, yaw orientation $\psi \in [-45^\circ, +45^\circ]$, and radial coordinates $(r, \theta)$.
- Publishes structured detections to `/vision/detected_objects` and annotated video to `/vision/annotated_image`.

---

## 6. Hardware Deployment, Launch Configuration & Linux Udev Rules

### 6.1 Linux Persistent `udev` Device Mapping

To guarantee deterministic USB serial mapping across host reboots, create `/etc/udev/rules.d/99-xarm.rules` on the host/Jetson:

```udev
# /etc/udev/rules.d/99-xarm.rules
# Permanent deterministic symlink for Single xArm 1S Manipulator
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="xarm_1", GROUP="dialout", MODE="0666"
```

Reload and trigger the rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/xarm_*
# Output: /dev/xarm_1 -> /dev/ttyUSB0
```

### 6.2 Full System Launch File (`ros2_ws/src/sorting_arm/launch/xarm.launch.py`)

The unified launch file starts the hardware driver, computer vision pipeline, and autonomous sorter simultaneously:

```bash
# Launch complete autonomous sorting pipeline:
ros2 launch sorting_arm xarm.launch.py auto_start:=true

# Or launch in simulation / dry-run mode (no hardware required):
ros2 launch sorting_arm xarm.launch.py dry_run:=true auto_start:=true
```

---

## 7. Verification Runbook & Diagnostics

### 7.1 System Bringup Checklist

```bash
# 1. Build and run development container (from project root)
./docker/run.sh

# 2. Inside container: clean build and source workspace
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. Run automated offline unit test suite (32 tests)
pytest src/sorting_arm/test/test_kinematics.py src/sorting_arm/test/test_vision_pipeline.py src/sorting_arm/test/test_calibration_and_features.py src/sorting_arm/test/test_camera_alignment.py

# 4. Launch full system
ros2 launch sorting_arm xarm.launch.py auto_start:=true
```

### 7.2 Telemetry & Topic Verification

- **Perception Detections**:
  ```bash
  ros2 topic echo /vision/detected_objects
  ```
  Confirm output displays detected cubes with arena coordinates $X \in [70, 330\text{ mm}]$, $Y \in [40, 180\text{ mm}]$, assigned arm `xarm_1`, and valid polar coordinates $r, \theta$.

- **Manipulator Joint Feedback**:
  ```bash
  ros2 topic echo /xarm_1/joint_states
  ```
  Confirm joint positions reported for all 6 servos within safe tick limits $[100, 900]$.

- **Trigger Sorting**:
  ```bash
  ros2 service call /autonomous_sorter/start std_srvs/srv/Trigger "{}"
  ```

- **Emergency Stop**:
  ```bash
  ros2 service call /xarm_1/emergency_stop std_srvs/srv/Trigger "{}"
  ```

---

## 8. Summary & Cross-Stepping Stone Linkage

Stepping Stone 4 completes the operational architecture of the single-arm sorting robot:

- **Arena Layout**: Grounded in the single-arm flat baseboard ($400\times 400\text{ mm}$), tabletop rectangular arena ($Z = 0.0\text{ mm}$), and Left Flank 2×2 tray from [`docs/01_arena_and_layout.md`](01_arena_and_layout.md).
- **Kinematic Actuation**: Executes closed-form IK equations, top-down radial approaches, and 5-phase minimum-jerk trajectories from [`docs/02_kinematics_and_grasp_planning.md`](02_kinematics_and_grasp_planning.md).
- **Computer Vision**: Powered by the 4-point ArUco metric homography, camera alignment tool (`align_camera.py`), and HSV segmentation from [`docs/03_computer_vision_and_calibration.md`](03_computer_vision_and_calibration.md).
- **Hardware Protocols**: Directly communicates using the binary packet protocol and servo calibrations from [`docs/05_hardware_bringup_and_firmware_control.md`](05_hardware_bringup_and_firmware_control.md).
