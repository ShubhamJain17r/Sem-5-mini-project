# Dual xArm 1s Color-Sorting Robot — Implementation Plan

## System Overview

Two xArm 1s robotic arms pick randomly scattered colored wooden cubes from an arena and sort them into a shared **2×2 Collection Box** (4 cells, one per color). Each arm covers one half of the arena (top/bottom). Arms rotate ~180° between pick and drop. An overhead USB camera detects all cubes and their positions before any arm moves. The system runs under **ROS 2** on a Jetson Nano or Raspberry Pi 5.

---

## Hardware

| Component | Spec |
|---|---|
| Arms | 2× xArm 1s |
| Compute | Jetson Nano / Raspberry Pi 5 |
| Camera | Logitech C270 or C920 (USB) |
| Objects | Wooden cubes, spray-painted in 4 colors |
| Colors | Red, Green, Blue, Yellow (HSV-separated, no calibration ambiguity) |
| Collection Box | 2 columns × 2 rows, one cell per color |
| Connectivity | WiFi — xArm built-in → ROS 2 xarm driver |

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

  Pick direction: arms reach RIGHT into arena
  Drop direction: arms rotate ~180° and reach LEFT into collection box
```

- Arms are positioned between the arena and collection box.
- Both arms rotate away from each other during the 180° pick-to-drop swing.
- Arena dimensions are determined after physically measuring each arm's reach radius.
- Camera is mounted directly overhead on a boom tripod, covering the full arena.
- 4 ArUco markers at arena corners handle automatic perspective calibration.

---

## Vision Pipeline

**Technology**: Pure OpenCV — no machine learning, no training.

```
Frame from camera
  → Detect 4 ArUco corner markers
  → Compute perspective transform → rectified top-down view
  → Convert to HSV
  → Threshold for each of 4 colors → 4 binary masks
  → Find contours per mask → filter by area
  → Compute centroid (pixel) per detected cube
  → Apply pixel→mm transform
  → Output: [{color, x_mm, y_mm}, ...]
```

**Color configuration** (`config/colors.yaml`): All HSV ranges stored in a YAML file. Adding or removing a color requires only editing this file — no code changes.

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
| `dashboard_node` | Runs rosbridge WebSocket server + HTTP server for web UI |

### Custom Messages

```
DetectedObject.msg    → string color, float32 x_mm, float32 y_mm
ArmTask.msg           → string color, float32 pick_x, float32 pick_y,
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
```

All pick heights (`pick_z`) are identical — cubes are uniform size on a flat surface.
All bin coordinates (`bin_x`, `bin_y`, `bin_z`) are hardcoded per cell per arm in `config/bin_coordinates.yaml`.

---

## Scheduler: Zone Mutex & Pipeline Logic

Two mutexes prevent collisions:
- **`arena_mutex`**: Only one arm in the arena at a time.
- **`box_mutex`**: Only one arm accessing the collection box at a time.

Both mutexes can be held simultaneously by **different arms** (one arm picks while the other drops).

**Pipeline pattern**:
```
xArm1 acquires arena_mutex → picks → releases arena_mutex
  → xArm2 acquires arena_mutex → picks
  → simultaneously xArm1 acquires box_mutex → drops → releases box_mutex
  → xArm2 releases arena_mutex → acquires box_mutex → drops
  → repeat
```

**Object assignment**:
- `y_mm < arena_midpoint` → xArm 1
- `y_mm >= arena_midpoint` → xArm 2
- Objects exactly on midline → assigned to xArm 1

---

## Collection Box Coordinate System

```
┌───┬───┐
│ R │ G │   R = Red   G = Green
├───┼───┤
│ B │ Y │   B = Blue  Y = Yellow
└───┴───┘
```

Bin cell positions are measured physically once and stored in `config/bin_coordinates.yaml`.
Each entry is a coordinate in the respective arm's base frame (mm):

```yaml
arm1_bins:
  red:    {x: ___, y: ___, z: ___}
  green:  {x: ___, y: ___, z: ___}
  blue:   {x: ___, y: ___, z: ___}
  yellow: {x: ___, y: ___, z: ___}

arm2_bins:
  # same structure, different values
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
