# Dual xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from an arena and sort them into a shared **2×2 Collection Box** (4 cells, one per color). Each arm covers one half of the arena (top/bottom). Arms rotate ~180° between pick and drop. An overhead USB camera detects all cubes and their positions before any arm moves. The system runs under **ROS 2** on a Jetson Nano or Raspberry Pi 5.
Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from an expanded, continuous **overlapping sector arena** and sort them into **two dedicated 2×2 Collection Boxes** (one box mounted behind each arm). 

The pickable workspace consists of two sweeping circular sectors (~120°–140° arc per arm, $R_{min}=10\text{ cm}$ to $R_{max}=26\text{ cm}$) that overlap across the center line. Arms rotate ~180°–220° to drop cubes into their respective rear boxes. 

An overhead USB camera mounted at 75–85 cm captures the entire **80 × 60 cm baseboard**. 4 ArUco markers at the baseboard corners provide real-time homography calibration. The system runs under **ROS 2** on a Jetson Nano or Raspberry Pi 5.

---

## Hardware

| Component | Spec |
|---|---|
| Arms | 2× xArm 1s |
| Arms | 2× xArm 1s (5-DOF / 6-DOF bus servo arms) |
| Compute | Jetson Nano / Raspberry Pi 5 |
| Camera | Logitech C270 or C920 (USB) |
| Objects | Wooden cubes, spray-painted in 4 colors |
| Colors | Red, Green, Blue, Yellow (HSV-separated, no calibration ambiguity) |
| Collection Box | 2 columns × 2 rows, one cell per color |
| Connectivity | WiFi — xArm built-in → ROS 2 xarm driver |
| Camera | Logitech C270 or C920 (USB) mounted at 75–85 cm height |
| Baseboard | 80 cm × 60 cm flat board with 4 outer ArUco markers |
| Objects | Wooden cubes (25 mm), spray-painted in 4 colors |
| Colors | Red, Green, Blue, Yellow (HSV-separated) |
| Collection Boxes | 2× separate 2×2 boxes (one dedicated box per arm, located at rear) |
| Arena Boundaries | Curved physical barriers along $R_{min} = 10\text{ cm}$ and $R_{max} = 26\text{ cm}$ arcs |
| Connectivity | WiFi / Serial — xArm SDK → ROS 2 driver |

---

## Field Layout

 
```
┌────────────────────────────────────────────────────────────────┐
│  ┌──────────────┐  ┌────────┐  ┌──────────────────────────┐   │
│  │  COLLECTION  │  │ xArm 1 │  │ ◤ ArUco         ArUco ◥  │   │
│  │     BOX      │  │        │  │                          │   │
│  │  ┌───┬───┐   │  └────────┘  │     TOP HALF             │   │
│  │  │ R │ G │   │              │     (xArm 1 zone)        │   │
│  │  ├───┼───┤   │              │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  │   │
│  │  │ B │ Y │   │  ┌────────┐  │     BOTTOM HALF          │   │
│  │  └───┴───┘   │  │ xArm 2 │  │     (xArm 2 zone)        │   │
│  └──────────────┘  │        │  │                          │   │
│                    └────────┘  │ ◣ ArUco         ArUco ◢  │   │
│                                └──────────────────────────┘   │
│                                           ▲                    │
│                                    Camera overhead             │
│                                    (boom tripod)               │
└────────────────────────────────────────────────────────────────┘
┌────────────────────────────────────────────────────────────────────────┐
│ ◤ ArUco (Corner 1)                                ArUco (Corner 2) ◥   │
│                                                                        │
│               ╭────────────────────────────────────────╮               │
│               │     CONTINUOUS OVERLAPPING ARENA       │               │
│          ╭────╯   (Arm 1 Sector ∩ Arm 2 Sector)        ╰────╮          │
│         ╱                                                    ╲         │
│        │      xArm 1 Zone                 xArm 2 Zone         │        │
│        │  (R: 10cm to 26cm)           (R: 10cm to 26cm)       │        │
│         ╲        ╭─────╮                 ╭─────╮             ╱         │
│          ╰───────┤ Rmin│                 │ Rmin├────────────╯          │
│                  ╰──┬──╯                 ╰──┬──╯                       │
│                     │                       │                          │
│                 ┌───┴────┐  (~25-30cm)  ┌───┴────┐                     │
│                 │ xArm 1 │◄────────────►│ xArm 2 │                     │
│                 └────────┘              └────────┘                     │
│                     │                       │                          │
│               (~180°-220°)            (~180°-220°)                     │
│                     ↓                       ↓                          │
│                ┌─────────┐             ┌─────────┐                     │
│                │ 2×2 BOX │             │ 2×2 BOX │                     │
│                │ (Arm 1) │             │ (Arm 2) │                     │
│                └─────────┘             └─────────┘                     │
│                                                                        │
│ ◣ ArUco (Corner 3)         ▲ Overhead Camera      ArUco (Corner 4) ◢   │
│                          (Boom @ 75–85 cm)                             │
└────────────────────────────────────────────────────────────────────────┘

  Pick direction: arms reach RIGHT into arena
  Drop direction: arms rotate ~180° and reach LEFT into collection box
  Pick direction: arms reach FORWARD & SIDES into their ~120°–140° sectors
  Drop direction: arms rotate ~180°–220° to the REAR into dedicated boxes
```

- Arms are positioned between the arena and collection box.
- Both arms rotate away from each other during the 180° pick-to-drop swing.
- Arena dimensions are determined after physically measuring each arm's reach radius.
- Camera is mounted directly overhead on a boom tripod, covering the full arena.
- 4 ArUco markers at arena corners handle automatic perspective calibration.
- **Baseboard**: 80 cm × 60 cm flat board; 4 ArUco markers at the outer corners map the entire board to a global $(X, Y)$ mm frame.
- **Overlapping Arena**: Arms are mounted side-by-side spaced 25–30 cm apart. Their ~120°–140° sweeping sectors overlap in the center, creating one large continuous pick zone.
- **Physical Curved Barriers**: Inner barrier at $R_{min} = 10\text{ cm}$ prevents cubes from landing in the mechanical blind spot directly under the arms; outer barrier at $R_{max} = 26\text{ cm}$ prevents unreachable scatter.
- **Drop Zones**: Two separate 2×2 collection boxes mounted behind each arm eliminate drop-zone contention entirely.
- **Camera Mount**: Single overhead USB webcam on a boom stand at 75–85 cm height capturing the entire 80×60 cm board with margin.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```
Frame from camera
  → Detect 4 ArUco corner markers
  → Compute perspective transform → rectified top-down view
Frame from camera (Overhead @ 75–85 cm)
  → Detect 4 ArUco corner markers on 80×60 cm baseboard
  → Compute perspective transform → rectified top-down view (global mm plane)
  → Convert to HSV
  → Threshold for each of 4 colors → 4 binary masks
  → Find contours per mask → filter by area
  → Compute centroid (pixel) per detected cube
  → Apply pixel→mm transform
  → Output: [{color, x_mm, y_mm}, ...]
  → Apply pixel → global mm transform: (X_global, Y_global)
  → Convert to Polar Frame relative to each arm base:
      ΔX = X_global - X_base,  ΔY = Y_global - Y_base
      r = sqrt(ΔX² + ΔY²),     θ = atan2(ΔY, ΔX)
  → Software sanity check: Verify 100 mm ≤ r ≤ 260 mm within valid sector arc
  → Output: [{color, x_mm, y_mm, r_mm, theta_deg, target_arm}, ...]
```

**Color configuration** (`config/colors.yaml`): All HSV ranges stored in a YAML file. Adding or removing a color requires only editing this file — no code changes.

**Color configuration** (`config/colors.yaml`): All HSV ranges stored in YAML.
**ArUco calibration**: Runs every frame. Automatically corrects for any camera shift without manual recalibration.

---

## ROS 2 Architecture

### Nodes

| Node | Role |
|---|---|
| `camera_node` | Publishes raw frames from USB webcam → `/camera/image_raw` |
| `vision_node` | On trigger: runs full detection pipeline → publishes `/detected_objects` |
| `scheduler_node` | Splits objects by y-midpoint, assigns tasks to each arm, manages zone mutexes |
| `arm_node` | Executes task queue for one arm (two instances, one per arm, parameterized by `arm_id`) |
| `vision_node` | On trigger: runs ArUco homography + HSV pipeline → publishes `/detected_objects` |
| `scheduler_node` | Splits objects by y-midpoint, assigns tasks to each arm, manages `arena_mutex` |
| `arm_node` | Executes task queue for one arm using Radial LUT approach (two instances: `arm_1`, `arm_2`) |
| `dashboard_node` | Runs rosbridge WebSocket server + HTTP server for web UI |

### Custom Messages

```
DetectedObject.msg    → string color, float32 x_mm, float32 y_mm
ArmTask.msg           → string color, float32 pick_x, float32 pick_y,
DetectedObject.msg    → string color, float32 x_mm, float32 y_mm, float32 r_mm, float32 theta_deg, string assigned_arm
ArmTask.msg           → string color, float32 pick_r, float32 pick_theta, float32 pick_z,
                         float32 bin_x, float32 bin_y, float32 bin_z, string arm_id
ArmStatus.msg         → string arm_id, string state, int32 tasks_remaining, bool grip_ok
```

### Topics and Services

```
/camera/image_raw          → camera_node publishes
/detected_objects          → vision_node publishes, scheduler_node subscribes
/arm1/task  /arm2/task     → scheduler_node publishes, arm_nodes subscribe
/arm1/status /arm2/status  → arm_nodes publish, dashboard_node subscribes
/vision/trigger            → scheduler_node calls to start a scan
/annotated_frame           → vision_node publishes annotated image for dashboard
```

---

## Arm Motion Sequence (per task)

### Distance-Based Pitch Lookup Table (LUT)
Because the xArm cannot reach horizontally with a 90° vertical pitch at distance:
- **Near** ($r \approx 10\text{–}15\text{ cm}$): Steep downward pitch ($\approx 40°\text{–}45°$)
- **Mid** ($r \approx 16\text{–}21\text{ cm}$): Angled approach pitch ($\approx 25°\text{–}30°$)
- **Far** ($r \approx 22\text{–}26\text{ cm}$): Shallow/horizontal pitch ($\approx 10°\text{–}15°$)

### Motion Execution Steps:
```
1. move_above(pick_x, pick_y)   → position gripper directly above target cube
2. open_gripper()
3. descend(pick_z)              → lower to cube height
4. close_gripper()
5. check_grip()                 → read servo position; if fail → return_home, retry
6. ascend()
7. retract_to_compact()         → pull arm to short configuration before rotating
8. rotate_180()                 → swing from arena side to collection box side
9. extend_to_bin(bin_x, bin_y)  → move above correct bin cell
10. descend(bin_z)
11. open_gripper()              → release cube
12. ascend()
13. return_home()
1. rotate_base(theta)            → rotate arm base joint to target azimuth angle
2. lookup_pitch(r)               → query pitch angle α from LUT for radius r
3. set_roll(0°)                  → lock wrist roll level with table (prevents table collisions)
4. move_standoff(r - 30mm, z)    → position gripper backed off radially along approach line
5. open_gripper(55mm)            → fully open 2-finger claw
6. advance_radially(r, pick_z)   → slide forward into cube contact position
7. close_gripper()               → grip cube (claw shape mechanically self-centers 25mm cube)
8. check_grip()                  → read servo feedback; if failed → abort, return home, retry
9. ascend_and_retract()          → lift slightly and retract radially to clear obstacles
10. retract_to_compact()         → pull arm into compact pose for safe rotation
11. rotate_rear(180°–220°)       → swing to dedicated rear 2×2 collection box
12. extend_to_bin(bin_x, bin_y)  → move over designated color cell
13. descend(bin_z)               → lower into cell
14. open_gripper()               → release cube
15. ascend()                     → clear container rim
16. return_home()                → ready pose
```

All pick heights (`pick_z`) are identical — cubes are uniform size on a flat surface.
All bin coordinates (`bin_x`, `bin_y`, `bin_z`) are hardcoded per cell per arm in `config/bin_coordinates.yaml`.

---

## Scheduler: Zone Mutex & Pipeline Logic
## Scheduler: Mutex & Pipelined Execution

Two mutexes prevent collisions:
- **`arena_mutex`**: Only one arm in the arena at a time.
- **`box_mutex`**: Only one arm accessing the collection box at a time.
Because each arm has its own dedicated collection box at the rear, **no `box_mutex` is needed**. Only the front/center picking arena requires collision protection:

Both mutexes can be held simultaneously by **different arms** (one arm picks while the other drops).
- **`arena_mutex`**: Only one arm operates in the shared picking arena at any moment.

**Pipeline pattern**:
**Pipeline Pattern (Zero Idling)**:
```
xArm1 acquires arena_mutex → picks → releases arena_mutex
  → xArm2 acquires arena_mutex → picks
  → simultaneously xArm1 acquires box_mutex → drops → releases box_mutex
  → xArm2 releases arena_mutex → acquires box_mutex → drops
  → repeat
xArm 1 acquires arena_mutex → picks in arena
  → xArm 1 finishes pick, releases arena_mutex → starts swing to rear Box 1
  → simultaneously, xArm 2 acquires arena_mutex → picks in arena
  → simultaneously, xArm 1 drops in private Box 1 (no lock needed!)
  → xArm 2 finishes pick, releases arena_mutex → starts swing to rear Box 2
  → xArm 1 re-acquires arena_mutex for next pick
  → repeat until all queues empty
```

**Object assignment**:
**Object Assignment**:
- `y_mm < arena_midpoint` → xArm 1
- `y_mm >= arena_midpoint` → xArm 2
- Objects exactly on midline → assigned to xArm 1
- Dead-zone rejection: Any cube with $r < 100\text{ mm}$ or $r > 260\text{ mm}$ or outside the angular arc is flagged and skipped.

---

## Collection Box Coordinate System

Two identical 2×2 collection boxes are fixed behind each arm:

```
┌───┬───┐
│ R │ G │   R = Red   G = Green
├───┼───┤
│ B │ Y │   B = Blue  Y = Yellow
└───┴───┘
    Arm 1 Rear Box                 Arm 2 Rear Box
┌───────────┬───────────┐      ┌───────────┬───────────┐
│ Red (R)   │ Green (G) │      │ Red (R)   │ Green (G) │
├───────────┼───────────┤      ├───────────┼───────────┤
│ Blue (B)  │ Yellow (Y)│      │ Blue (B)  │ Yellow (Y)│
└───────────┴───────────┘      └───────────┴───────────┘
```

Bin cell positions are measured physically once and stored in `config/bin_coordinates.yaml`.
Each entry is a coordinate in the respective arm's base frame (mm):
Coordinates are measured once physically and stored in `config/bin_coordinates.yaml`:

```yaml
arm1_bins:
  red:    {x: ___, y: ___, z: ___}
  green:  {x: ___, y: ___, z: ___}
  blue:   {x: ___, y: ___, z: ___}
  yellow: {x: ___, y: ___, z: ___}

arm2_bins:
  # same structure, different values
  red:    {x: ___, y: ___, z: ___}
  green:  {x: ___, y: ___, z: ___}
  blue:   {x: ___, y: ___, z: ___}
  yellow: {x: ___, y: ___, z: ___}
```

---

## Web Dashboard

**Stack**: `rosbridge_server` (WebSocket) + HTML/CSS/JS frontend served over HTTP.

**Layout**:
- **Left panel**: Live annotated camera feed — bounding boxes, color labels, assigned arm, pick order.
- **Right panel**: Arm 1 state, Arm 2 state, task queue count, grip success/failure log.

No external frameworks — vanilla JS connects to rosbridge WebSocket and renders updates in real time.

---

## Repository Structure

```
sem3-robotics/
├── vision/
│   ├── aruco_calibration.py
│   ├── color_detect.py
│   └── pipeline.py
├── ros2_ws/src/
│   ├── camera_node/
│   ├── vision_node/
│   ├── scheduler_node/
│   ├── arm_node/
│   ├── dashboard_node/
│   └── custom_msgs/
├── dashboard/
│   ├── index.html
│   ├── style.css
│   └── app.js
├── config/
│   ├── colors.yaml
│   └── bin_coordinates.yaml
└── README.md
```
