# 🌐 Web Mission Control Dashboard & Digital Twin Operator Guide
### Single xArm 1S Dual-Color (Red & Green) Sorting Workcell

---

> [!CAUTION]
> ## ⚠️ THE ABSOLUTE RULE OF EXECUTION: STRICT TWO-ENVIRONMENT SEPARATION
> This rule is fundamental and must NEVER be violated under any circumstances:
> 
> 1. 🐳 **ANY command related to ROS 2 MUST be executed inside the Docker container** (`./docker/run.sh`).
>    - Examples: `colcon build`, `ros2 launch ...`, `ros2 run ...`, `ros2 topic ...`, `rviz2`, ROS test suites (`pytest`).
> 2. 🐍 **ANY standalone Python command MUST be executed inside the Python virtual environment** (`source venv/bin/activate`).
>    - Examples: `python3 tools/autonomous_dashboard.py`, `python3 tools/run_autonomous_system.py`, `python3 tools/web_config/server.py`, `python3 vision/*.py`.
> 
> 🚫 **NEVER** run ROS 2 commands directly on the host operating system.  
> 🚫 **NEVER** run project Python scripts without activating the Python virtual environment (`venv`).

---

## 📌 1. System Overview

The **Web Mission Control Dashboard** (`tools/autonomous_dashboard.py` + `tools/dashboard_ui/index.html`) is a zero-dependency, presentation-ready browser interface designed for live demonstrations, operator control, and real-to-sim digital twin synchronization.

### Key Capabilities
- **Background Hardware Watchdog**: Continuously probes video capture devices, physical xArm USB serial/HID interfaces, and Linux permissions.
- **Dynamic Camera Switcher**: Automatically enumerates video inputs, differentiates between laptop webcams and external overhead cameras, and allows hot-switching without restarting the server.
- **Hand-Held Camera Assist HUD**: 4-point ArUco alignment reticles (`M0`..`M3`), real-time camera height calculation ($H \approx 60\text{ cm}$), and perpendicular depression tilt gauge ($\theta \approx 90^\circ$).
- **"Snap & Lock" Stationary Frame Buffer**: Lets an operator hold a non-permanent camera overhead, lock the rectified scene with one click (or spacebar), and safely set the camera down while autonomous sorting proceeds.
- **Golden Zone Perception & Dynamic Roll**: HSV color segmentation in the kinematic sweet spot (Rows 7..9, Cols 2..4) with dynamic wrist roll ($130$ ticks for horizontal vs $500$ ticks for square/vertical).
- **Physical-to-Digital Twin Mirroring**: Synchronously reflects physical detected cubes and real-time arm trajectories into both the web 3D visualizer and ROS 2 RViz simulation.

---

## 🚀 2. Quickstart: Launching the Dashboard

### Step 1: Open Terminal & Activate Python Virtual Environment
On your host machine terminal:
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Run on host machine
# ==============================================================================
cd ~/Projects/Sem-5-mini-project

# Activate host venv (run ./setup_venv.sh first if not already configured)
source venv/bin/activate
```

### Step 2: Ensure USB Hardware Permissions
Ensure your host user has access to camera and serial/HID nodes:
```bash
sudo chmod 666 /dev/hidraw* /dev/ttyUSB* /dev/video* 2>/dev/null || true
```

### Step 3: Launch the Dashboard Server

#### Option A: Simulation / Dry-Run Mode (No Physical xArm Required)
Simulates arm motion safely, connects to available camera (or synthetic frames), and mirrors objects:
```bash
python3 tools/autonomous_dashboard.py --dry-run
```

#### Option B: Live Hardware Mode (Physical xArm + Connected Camera)
Commands physical bus-servos over USB HID / UART:
```bash
python3 tools/autonomous_dashboard.py
```

#### Optional CLI Arguments
| Flag | Description | Default |
| :--- | :--- | :--- |
| `--port <PORT>` | Web server port | `8080` (auto-falls back to `8081`, `8082`, etc.) |
| `--camera <IDX>` | Starting camera device index | `0` (`/dev/video0`) |
| `--serial_port <PORT>` | xArm serial port | `auto` (auto-detects HID `/dev/hidraw*` or serial `/dev/ttyUSB*`) |
| `--dry-run` | Safety mode: suppresses physical servo motion | `False` |

### Step 4: Open the Web Console
Open any modern web browser to:
```text
http://localhost:8080
```

---

## 🎥 3. Camera Setup & Video Device Switching

On modern laptops, `/dev/video0` is typically the **integrated webcam**, while an external overhead camera connects as `/dev/video2`, `/dev/video4`, etc.

```text
┌────────────────────────────────────────────────────────────────────────┐
│ TOP BAR: VIDEO DEVICE SELECTOR                                         │
│ Video Device: [ Camera 2: USB 2.0 Camera (/dev/video2) [Overhead]  ▼ ] [🔄] │
└────────────────────────────────────────────────────────────────────────┘
```

1. **Integrated Webcam Warning**:
   If the dashboard detects that the active feed is from an internal webcam, a yellow notice appears:
   > ⚠️ **Laptop Webcam Active**: You are currently capturing from your integrated laptop camera. If using an overhead USB camera, select it from the Video Device dropdown above.
2. **Hot-Switching Video Source**:
   Select the desired camera from the **Video Device** dropdown. The backend releases the previous device, re-binds to the new device via V4L2, and resumes streaming within 500 ms without dropping client connections.
3. **Rescan Devices (`🔄`)**:
   If you plug in a USB camera *after* the dashboard has already started, click the **`🔄` (Rescan)** button to probe new video devices immediately.

---

## 🔍 4. Reading Hardware Status Badges

The toolbar displays live hardware status badges polled every 400 ms:

| Badge | State | Meaning & Action |
| :--- | :--- | :--- |
| **Camera** | `🟢 /dev/videoX (Active)` | Camera is streaming valid frames at $\sim 25\text{ FPS}$. |
| | `🔴 No Signal` | Video node disconnected or permissions missing. |
| **xArm** | `🟢 Connected [ttyUSB0 (SERIAL)]` | Physical robot detected and communication verified over USB. |
| | `🔴 Disconnected (Simulation)` | No physical robot detected. The system operates in safe Dry-Run mode. |
| **Mode** | `🟢 Live Execution` | Servo commands dispatch to physical LewanSoul servos. |
| | `🟡 Dry-Run (Safety)` | Trajectories are solved and animated without powering physical motors. |

> [!TIP]
> **Fixing Disconnected xArm Status**:
> If your xArm is plugged in via USB but shows **Disconnected**, verify permissions:
> ```bash
> sudo chmod 666 /dev/hidraw* /dev/ttyUSB*
> ```
> Then click **Re-Probe** in the Camera HUD bar.

---

## 📐 5. Hand-Held Camera Assist HUD & "Snap & Lock"

When the camera is mounted on a flexible tripod or held by hand above the arena, use the **Optical Assist HUD**:

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│  [●] M0 (TL)    [●] M1 (TR)    [●] M2 (BL)    [●] M3 (BR)  │ Height: 60.5 cm │ Tilt: 89.2° │ [🔒 Freeze Frame] │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 1. 4-Corner ArUco Alignment Indicators
- **`M0 (TL)`**, **`M1 (TR)`**, **`M2 (BL)`**, **`M3 (BR)`**: Turn **green** when the respective fiducial marker (`DICT_4X4_50`, IDs 0..3) is detected in the field of view.
- All 4 markers must be green for metric homography rectification.

### 2. Height & Tilt Extrinsics
- **Height ($H_c$)**: Optimal operating height is **$55\text{ cm}$ to $70\text{ cm}$** (nominal $60\text{ cm}$). Shows green when within tolerance, yellow when too low or too high.
- **Tilt ($\theta$)**: Camera depression angle. Optimal is **$90^\circ$ nadir** (tolerance $\ge 72^\circ$).

### 3. "Snap & Lock" Workflow
1. Position the camera above the arena until all 4 marker dots turn **green**.
2. Press **`Spacebar`** (or click **`📸 Snap Frame`**).
3. The dashboard captures the current high-resolution frame, freezes it into stationary memory, and performs HSV color segmentation on the frozen scene.
4. **The operator can now safely set the camera down.** The robot executes pick-and-place from the locked target coordinates.
5. To unlock and return to the live camera feed, click **`🔓 Live Feed`** (or press Spacebar again).

---

## 📺 6. Viewport Displays

Use the tab switcher above the video viewport to toggle perspectives:

1. **Rectified Grid (`2px/mm`)**:
   - Homography-warped metric plane between ArUco centers ($600 \times 500\text{ px}$).
   - Renders the **Golden Pickup Zone** boundary (Rows 7..9, Cols 2..4).
   - Draws oriented bounding boxes, center coordinates, and color tags on detected blocks.
2. **Raw Camera Feed**:
   - Full field of view showing raw camera perspective with alignment crosshairs.
3. **3D Digital Twin Simulation**:
   - Real-time 3D rendered projection showing current arm joint angles ($\theta_1\text{--}\theta_6$), claw state, table grid, drop container boxes, and mirrored physical cubes.

---

## 📦 7. Target Queue & Trajectory Execution

### Golden Zone Target Queue
Detected craft blocks are automatically sorted in **Nearest-First** radial order ($r = \sqrt{X_{\text{arm}}^2 + Y_{\text{arm}}^2}$) to prevent outer-reach collisions:
- **`#`**: Pick sequence index.
- **`Color`**: `RED` (routed to Red Box @ $650$ ticks) or `GREEN` (routed to Green Box @ $354$ ticks).
- **`Arena (X, Y)`**: Metric table coordinates in mm.
- **`Reach (r)`**: Radial distance from arm shoulder ($140\text{--}260\text{ mm}$ sweet spot).
- **`Roll Tick`**: Dynamic wrist roll orientation ($130$ ticks for horizontal vs $500$ ticks for vertical/square).
- **`Container`**: Target container destination.

### 7-Phase Trajectory Stepper
As the arm executes, the progress bar highlights the active motion phase:
```text
[ 1. Hover (Z=75) ] ➔ [ 2. Plunge (Z=22) ] ➔ [ 3. Grip (520) ] ➔ [ 4. Lift (Z=90) ] ➔ [ 5. Rear Transit ] ➔ [ 6. Release (250) ]
```

### Controls
- **`▶ Start Sort`** (Hotkey: `S`): Automatically sorts all items in queue sequentially.
- **`Sort Next`**: Picks and sorts only the first item in the queue.
- **`Home Pose`** (Hotkey: `H`): Returns arm to neutral upright tucked position.
- **`🛑 E-Stop`** (Hotkey: `Esc`): Immediately cuts motion and unloads servo torque.

---

## 🌐 8. Simultaneous Execution with ROS 2 RViz Digital Twin

To run the web dashboard alongside the full ROS 2 Humble RViz simulation:

### Terminal 1: Launch Web Dashboard (Host venv)
```bash
# ==============================================================================
# [HOST PYTHON VIRTUAL ENVIRONMENT] Host Terminal
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================
source venv/bin/activate
python3 tools/autonomous_dashboard.py --dry-run
```

### Terminal 2: Launch ROS 2 Digital Twin Simulation (Inside Docker)
```bash
# ==============================================================================
# [HOST TERMINAL -> DOCKER] Host Terminal
# Working Directory: ~/Projects/Sem-5-mini-project (Repository Root)
# ==============================================================================
./docker/run.sh ros2 launch sorting_arm simulation.launch.py gui:=true
```

When you click **`Snap Frame`** on the web dashboard:
1. The dashboard detects the physical blocks via computer vision.
2. It publishes the coordinates and colors over ROS 2 topic `/vision/detected_objects`.
3. The ROS 2 `arena_visualizer_node` immediately spawns the identical 3D cubes inside RViz2 at the exact physical $(X, Y)$ positions!

---

## ⌨️ 9. Keyboard Shortcuts Cheat Sheet

| Key | Action | Description |
| :---: | :--- | :--- |
| **`Space`** | **Snap / Freeze Frame** | Freezes the aligned camera frame and queues detected blocks |
| **`S`** | **Start Sort All** | Begins autonomous pick-and-place sequence for all queued objects |
| **`H`** | **Home Pose** | Moves arm to neutral safe home position |
| **`Esc`** | **Emergency Stop** | Immediately halts all motion and opens claw |

---

## 🔧 10. Troubleshooting & FAQ

### Q1: The browser page says "Cannot connect" or loads a blank page
- Check that the server is running in terminal: `python3 tools/autonomous_dashboard.py`.
- If port `8080` was occupied, look at the terminal startup output: the server automatically tries ports `8081`, `8082`, etc. Open the exact URL displayed in terminal.

### Q2: Camera shows black screen or "No Signal"
- If running with an external webcam, select the correct device from the **Video Device** dropdown (e.g., `Camera 2`).
- Check permissions: `sudo chmod 666 /dev/video*`.
- Ensure no other application (e.g., Zoom, Cheese, Chrome) is holding an exclusive lock on the camera device.

### Q3: xArm status shows "Disconnected (Simulation)" even though USB is plugged in
- Verify that the blue power switch on the xArm base is toggled **ON** and the 7.4V / 5A power supply is connected.
- Grant USB permissions: `sudo chmod 666 /dev/hidraw* /dev/ttyUSB*`.
- Click **Re-Probe** in the web dashboard.

### Q4: ArUco markers are not turning green
- Ensure camera height is around $55\text{--}70\text{ cm}$ directly overhead.
- Ensure adequate diffuse lighting without harsh specular glares on vinyl markers.
- Verify marker IDs: Top-Left is ID 0, Top-Right is ID 1, Bottom-Left is ID 2, Bottom-Right is ID 3 (`DICT_4X4_50`).
