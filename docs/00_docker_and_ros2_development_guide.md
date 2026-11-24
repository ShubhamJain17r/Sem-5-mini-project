# Stepping Stone 0: Docker & ROS 2 Humble Development Guide
### Single xArm 1S Color-Sorting Robot

---

## 📌 Document Overview

This engineering guide provides the complete development standard for building, writing, and executing ROS 2 packages within the project's isolated Docker environment. 

It eliminates global installation conflicts on modern Linux hosts (Ubuntu 24.04 / 26.04), provides hardware acceleration for Intel graphics and NVIDIA Jetson (ARM64), and defines the exact workspace scaffolding for the project's ROS 2 node graph.

---

## 1. Architectural Philosophy: The Hybrid Workflow

The project uses a **hybrid development model** designed for maximum productivity:

```text
┌────────────────────────────────────────────────────────────────────────┐
│ HOST LAPTOP (Ubuntu 26.04 / Any Linux)                                 │
│  - Edit code comfortably in VS Code, Cursor, or your favorite IDE.     │
│  - Manage Git branches, reviews, and commits.                          │
│  - All source files persist locally in ~/Projects/Sem-5-mini-project.  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Live Volume Mount (/workspace)
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│ DOCKER CONTAINER (Ubuntu 22.04 LTS + ROS 2 Humble Desktop)             │
│  - Compiles packages with `colcon build`.                              │
│  - Runs ROS 2 nodes, launch files, and RViz2 visualizers.              │
│  - Uses Intel GPU (`/dev/dri`) for hardware-accelerated OpenGL.        │
│  - Directly accesses physical USB hardware (/dev/video*, /dev/ttyUSB*).│
└────────────────────────────────────────────────────────────────────────┘
```

> [!TIP]
> **No Rebuilding Needed for Code Changes:** Because the repository root is live-mounted into `/workspace`, edits made in your host editor update inside the container immediately. You do **not** re-run `./docker/build.sh` when changing Python or C++ code.

---

## 2. Step-by-Step: Starting a ROS 2 Project in Docker

### Step 1: Start the Docker Container
From your repository root on your laptop terminal:
```bash
./docker/run.sh
```
*Your prompt changes to `shubham@Shubham:/workspace$`.*  
You are now inside the isolated Ubuntu 22.04 + ROS 2 Humble environment with Intel GPU and hardware access enabled.

---

### Step 2: Initialize the ROS 2 Workspace Structure
Inside the container, navigate to `/workspace` and create the `ros2_ws` directory layout:
```bash
mkdir -p /workspace/ros2_ws/src
cd /workspace/ros2_ws/src
```
*(Because `/workspace` is mounted from your host, `ros2_ws/src` immediately appears on your laptop filesystem as well).*

---

### Step 3: Scaffold Your ROS 2 Packages
Inside `/workspace/ros2_ws/src`, create packages according to their language and role:

#### Option A: Python Node Package (e.g. Driver, Vision, Scheduler)
```bash
ros2 pkg create --build-type ament_python sorting_arm \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

#### Option B: Custom Interfaces Package (Messages & Services)
```bash
ros2 pkg create --build-type ament_cmake sorting_interfaces \
  --dependencies rosidl_default_generators
```

#### Option C: C++ High-Performance Package (e.g. Kinematics Solver)
```bash
ros2 pkg create --build-type ament_cmake sorting_kinematics \
  --dependencies rclcpp std_msgs
```

---

### Step 4: Write Your Node Code (On Your Laptop in VS Code)
Do **not** struggle with terminal text editors inside Docker. Work directly on your laptop:
1. Open VS Code on your laptop:
   ```bash
   code ~/Projects/Sem-5-mini-project
   ```
2. In the explorer pane, open `ros2_ws/src/sorting_arm/sorting_arm/`.
3. Create your Python node (for example, `arm_node.py`):
   ```python
   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class ArmNode(Node):
       def __init__(self):
           super().__init__('arm_node')
           self.get_logger().info('xArm 1s Node Initialized inside Docker!')

   def main(args=None):
       rclpy.init(args=args)
       node = ArmNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
4. Save the file. It is immediately available inside Docker at `/workspace/ros2_ws/src/sorting_arm/sorting_arm/arm_node.py`.

---

### Step 5: Register the Node Executable
In your laptop editor, open `ros2_ws/src/sorting_arm/setup.py`. Register your node inside `entry_points['console_scripts']`:

```python
entry_points={
    'console_scripts': [
        'arm_node = sorting_arm.arm_node:main',
    ],
},
```

Make sure any launch files or resource files are included in `data_files` if required.

---

### Step 6: Build the Workspace (Inside Container)
Switch back to your container terminal:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
```

> [!IMPORTANT]
> **Why `--symlink-install`?**  
> For Python packages, `--symlink-install` links directly to your source files in `src/`. This means future edits to `.py` files take effect immediately **without needing to re-run `colcon build`**!

---

### Step 7: Source the Workspace Overlay (Inside Container)
Activate the freshly built packages:
```bash
source /workspace/ros2_ws/install/setup.bash
```
*(The container's `~/.bashrc` is already configured to source this automatically on all newly opened terminal sessions).*

---

### Step 8: Run Your Node
Verify execution with standard ROS 2 commands:
```bash
ros2 run sorting_arm arm_node
```
You should see:
```text
[INFO] [arm_node]: xArm 1s Node Initialized inside Docker!
```

---

## 3. Multi-Terminal Management

Robotics applications require multiple terminals running concurrently.

### Terminal 1: Primary Runner
Runs the main container:
```bash
./docker/run.sh
```

### Terminal 2, 3, 4: Attaching Additional Shells
To open additional shells inside the **same running container**, open a new terminal tab on your laptop (`Ctrl + Shift + T`) and run:
```bash
docker exec -it xarm_sorting_env bash
```

Inside this attached shell, your ROS 2 environment and built packages are already sourced and ready to run.

---

## 4. Hardware Verification Commands (Inside Docker)

### 1. Camera Verification
```bash
# Check detected video devices
ls -l /dev/video*

# Query camera capabilities (Logitech C270 / C920)
v4l2-ctl --device=/dev/video0 --all

# Launch v4l2 ROS 2 camera node
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
```

### 2. xArm USB-to-UART Serial Verification
```bash
# Check detected USB serial converters
ls -l /dev/ttyUSB* /dev/ttyACM*

# Quick Python loopback / baud rate check
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1); print('Serial port open:', s.is_open); s.close()"
```

### 3. Intel GPU Hardware Acceleration Check
```bash
# Verify OpenGL direct hardware rendering
glxinfo -B
```
Confirm `direct rendering: Yes` and `OpenGL vendor string: Intel`.

---

## 5. Deployment on Jetson Orin / Xavier

When deploying to the onboard Jetson computer, the workflow remains identical:

1. Clone repository to Jetson.
2. Run `./docker/build.sh` (Docker pulls the ARM64 ROS 2 Humble base image).
3. Run `./docker/run.sh` to enter the container.
4. Build packages with memory protection:
   ```bash
   colcon build --symlink-install --parallel-workers 2
   ```

---

## 6. Development Checklist

| Phase | Action | Location |
| :--- | :--- | :--- |
| **Setup** | Launch Docker container (`./docker/run.sh`) | Laptop Terminal |
| **Code** | Write Python/C++ code in VS Code | Laptop Host |
| **Build** | Compile (`colcon build --symlink-install`) | Container Terminal |
| **Source** | Activate overlay (`source install/setup.bash`) | Container Terminal |
| **Inspect** | Telemetry check (`ros2 topic list`, `ros2 topic echo ...`) | Attached Terminal (`docker exec`) |
| **Visualize** | 3D visualizer (`rviz2`) | Container Terminal |
| **Commit** | Stage & commit changes (`git commit`) | Laptop Host Terminal |

---

## 7. Master Autonomous Pipeline & Self-Healing Execution

The repository provides a single, unified master CLI that runs the complete autonomous system:
[`tools/run_autonomous_system.py`](file:///home/shubham/Projects/Sem-5-mini-project/tools/run_autonomous_system.py)

### Presetup Commands:
```bash
# 1. Build and run Docker (Host Terminal)
./docker/build.sh
./docker/run.sh

# 2. Inside Container Terminal: Build ROS 2 workspace
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. Launch Master Autonomous Sorting Pipeline
python3 tools/run_autonomous_system.py
```

### Command Flags:
- `--dry-run`: Simulates the entire vision perception, extrinsics, filtering, and kinematic trajectories without physically driving manipulator motors.
- `--step`: Prompts for user `[Enter]` before picking each target cube (ideal for verification and grading).
- `--auto`: Continuously sorts all detected objects autonomously.
- `--camera <idx>`: Specifies video capture device index (default: `4`).
- `--port <dev>`: Specifies xArm controller port (default: `auto`).

### Self-Healing & Interactive Diagnostics:
1. **Stage 1 (Hardware & Access)**:
   - If `/dev/video*` or `/dev/hidraw*` permissions are restricted, the program **does not crash**.
   - It prints the exact `sudo chmod 666 ...` command in the terminal and waits for `[Enter]` in a retry loop.
2. **Stage 2 (Vision Extrinsics & Marker Verification)**:
   - Measures camera mounting height $H_c$, Euclidean distance $D_c$, and depression angle $\theta_{\text{dep}}$.
   - Verifies all 4 ArUco markers (M0, M1, M2, M3). If tilted or displaced, prints exact degrees and rotation direction to adjust.
3. **Stage 3 (Perception & Spatial Filtering)**:
   - Warps arena to $2.0\text{ px/mm}$ metric grid with 3D parallax height correction.
   - Enforces the **Golden Pickup Zone (Rows 7..9, Cols 2..4, $X \in [75, 275\text{ mm}]$, $Y \in [100, 250\text{ mm}]$, $r \in [140, 260\text{ mm}]$)**.
   - Detects block aspect ratio orientation and commands dynamic wrist roll ($130\text{ ticks}$ for horizontal blocks, $500\text{ ticks}$ for vertical/square).
4. **Stage 4 (Scheduling & Pick-and-Place)**:
   - Sorts targets using Nearest-First radial reach ordering.
   - Applies empirical gravity sag compensation ($+10.0\text{ mm} + 0.05 \times \max(0, r - 150)$) and executes a pure top-down vertical plunge trajectory ($Z=75\text{ mm} \to Z=22\text{ mm}$).
   - Clamps firmly at 520 ticks and transits cubes to Rear Flank Red and Green collection boxes via collision-free overhead rear-flip trajectories.
