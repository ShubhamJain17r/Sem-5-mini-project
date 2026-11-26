# 🐳 Complete Docker & ROS 2 Humble User Manual
### Single xArm 1S Color-Sorting Robot

---

## 📌 Table of Contents
1. [Mental Model: Host vs. Container](#1-mental-model-host-vs-container)
2. [Quickstart (First-Time Setup)](#2-quickstart-first-time-setup)
3. [Daily Development Workflow](#3-daily-development-workflow)
4. [Managing Multiple Terminals](#4-managing-multiple-terminals)
5. [ROS 2 Workspace & Build Lifecycle](#5-ros-2-workspace--build-lifecycle)
6. [GUI & Hardware-Accelerated 3D Visualization](#6-gui--hardware-accelerated-3d-visualization)
7. [Physical Hardware Access (Cameras & Serial Servos)](#7-physical-hardware-access-cameras--serial-servos)
8. [Deploying to NVIDIA Jetson (ARM64)](#8-deploying-to-nvidia-jetson-arm64)
9. [Distributed Multi-Machine Networking (Laptop + Jetson)](#9-distributed-multi-machine-networking-laptop--jetson)
10. [Troubleshooting & FAQ](#10-troubleshooting--faq)
11. [Command Cheat Sheet](#11-command-cheat-sheet)

---

> [!CAUTION]
> ## ⚠️ THE ABSOLUTE RULE OF EXECUTION: STRICT TWO-ENVIRONMENT SEPARATION
> 1. 🐳 **ANY command related to ROS 2 MUST run inside Docker** (`./docker/run.sh`).
>    - Examples: `colcon build`, `ros2 launch ...`, `ros2 run ...`, `ros2 topic ...`, `rviz2`.
> 2. 🐍 **ANY Python command MUST run in the Python virtual environment** (`source venv/bin/activate`).
>    - Examples: `python3 tools/autonomous_dashboard.py`, `python3 tools/web_config/server.py`, `python3 vision/*.py`.
> 
> **NEVER** run ROS 2 commands directly on the host operating system.  
> **NEVER** run project Python scripts without activating `venv`.

---

## 1. Mental Model: Host vs. Container

Understanding what runs where avoids 99% of common beginner issues:

```text
┌────────────────────────────────────────────────────────────────────────┐
│ HOST LAPTOP (Ubuntu 26.04 / Any Linux)                                 │
│  - Edit files with VS Code / Cursor / nano                             │
│  - Run git commands (git add, git commit, git push)                    │
│  - Stores all persistent files in: ~/Projects/Sem-5-mini-project       │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Live Volume Mount (Instant Sync)
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│ DOCKER CONTAINER (Ubuntu 22.04 LTS + ROS 2 Humble)                     │
│  - Where code is built (`colcon build`)                                │
│  - Where ROS 2 nodes, launch files, and RViz2 run                      │
│  - Uses Intel GPU (/dev/dri) directly for OpenGL rendering             │
│  - Accesses webcams (/dev/video*) & xArms (/dev/ttyUSB*)               │
└────────────────────────────────────────────────────────────────────────┘
```

> [!IMPORTANT]
> **Live Volume Mount**: Your project directory is mounted inside at `/workspace`.  
> When you edit code in your editor on the laptop, the changes are **instantly updated inside Docker**.  
> You **never** need to rebuild the Docker image after modifying Python scripts, launch files, or C++ source code!

---

## 2. Quickstart (First-Time Setup)

From your project root on your laptop:

```bash
# 1. Ensure you are in the repository root
cd ~/Projects/Sem-5-mini-project

# 2. Build the project Docker image (only needed once or after modifying Dockerfile)
./docker/build.sh
```

This generates the Docker image `xarm-sorting:humble`:
* Downloads **ROS 2 Humble Desktop**.
* Sets up **Intel OpenGL (Mesa)** graphics drivers.
* Configures Python dependencies (`opencv-python`, `pyserial`, `numpy`, `scipy`).
* Creates the `developer` user with your exact host `UID:GID` (1000:1000) and `dialout`/`video`/`render` permissions.

---

## 3. Daily Development Workflow

### Step 1: Open Your Code Editor on Host
```bash
cd ~/Projects/Sem-5-mini-project
code .
```

### Step 2: Launch the ROS 2 Container
```bash
./docker/run.sh
```
Your prompt will transition into the container:
```bash
shubham@Shubham:/workspace$
```

### Step 3: Exit When Finished
```bash
exit
```
The container safely shuts down. All code, logs, and build products remain saved on your host.

---

## 4. Managing Multiple Terminals

In robotics, you frequently need several terminals running simultaneously (e.g., Terminal 1 runs the camera driver, Terminal 2 runs the robot controller, Terminal 3 monitors topics).

### Primary Terminal (Runs the Container)
```bash
./docker/run.sh
```

### Additional Terminals (Joining the Active Container)
Open a new terminal tab on your host (`Ctrl + Shift + T`) and run:
```bash
docker exec -it xarm_sorting_env bash
```
*(You can open as many additional terminal tabs as you need using this command).*

---

## 5. ROS 2 Workspace & Build Lifecycle

Inside the container, all ROS 2 packages live in `/workspace/ros2_ws/src`.

### Building Packages
```bash
cd /workspace/ros2_ws

# Clean build
colcon build --symlink-install
```

### Sourcing the Built Overlay
```bash
source /workspace/ros2_ws/install/setup.bash
```
*(Note: Container shells automatically source this overlay upon startup if it exists).*

### Creating a New ROS 2 Package
```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python my_package_name --dependencies rclpy std_msgs sensor_msgs
```

---

## 6. GUI & Hardware-Accelerated 3D Visualization

Thanks to the Intel Iris/UHD hardware pass-through (`/dev/dri`), RViz2 and OpenCV graphical windows run at native hardware speed.

### Launching RViz2 (Inside Container)
```bash
rviz2
```

### Verifying Hardware Acceleration (Inside Container)
```bash
glxinfo -B
```
Expected output confirms:
* `direct rendering: Yes`
* `OpenGL vendor string: Intel`
* `Accelerated: yes`

---

## 7. Physical Hardware Access (Cameras & Serial Servos)

`./docker/run.sh` automatically maps all active camera and USB serial devices into the container.

### 1. Overhead USB Webcam (Logitech C270 / C920)
1. Plug the webcam into your laptop USB port.
2. Inside the container, check detected video nodes:
   ```bash
   v4l2-ctl --list-devices
   ls -l /dev/video*
   ```
3. Test streaming:
   ```bash
   ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0"
   ```

### 2. Single xArm 1S Serial Controller
1. Plug the xArm USB-UART controller into your laptop.
2. Inside the container, check detected serial devices:
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```
3. Test serial communication via Python:
   ```bash
   python3 -c "import serial; print(serial.__file__)"
   ```

> [!NOTE]
> If you unplug and plug a USB device back in, restart the container via `./docker/run.sh` so Docker can re-bind the new `/dev/` node.

---

## 8. Deploying to NVIDIA Jetson (ARM64)

The project Docker setup is 100% multi-architecture (`linux/amd64` and `linux/arm64`).

### On the Jetson:
1. Clone the repository:
   ```bash
   git clone <repo-url>
   cd Sem-5-mini-project
   ```
2. Build the Docker image on the Jetson:
   ```bash
   ./docker/build.sh
   ```
   *(Docker automatically pulls the ARM64 variant of ROS 2 Humble).*
3. Run the container:
   ```bash
   ./docker/run.sh
   ```
4. Build with memory throttling (prevents Jetson RAM exhaustion):
   ```bash
   cd /workspace/ros2_ws
   colcon build --symlink-install --parallel-workers 2
   ```

---

## 9. Distributed Multi-Machine Networking (Laptop + Jetson)

Because both the laptop and Jetson containers execute with `--net=host`, ROS 2 DDS automatically discovers nodes across your local Wi-Fi or Ethernet network:

```text
┌──────────────────────────────────────┐          ┌──────────────────────────────────────┐
│ LAPTOP (Desk / Visualizer)           │          │ JETSON (Onboard Robot / Controller)  │
│  - Runs RViz2                        │  Wi-Fi   │  - Connected to xArm serial cables   │
│  - Displays vision detections        │ ◄──────► │  - Runs motor motion profiles        │
│  - Receives /arm1/status             │   DDS    │  - Publishes /arm1/status            │
└──────────────────────────────────────┘          └──────────────────────────────────────┘
```

### Setup:
1. Connect both laptop and Jetson to the same Wi-Fi router or via an Ethernet switch.
2. (Optional) Set a matching Domain ID on both machines:
   ```bash
   export ROS_DOMAIN_ID=42
   ```
3. Launch hardware on the Jetson:
   ```bash
   ros2 launch sorting_arm xarm.launch.py auto_start:=true
   ```
4. Open RViz or monitor topics on your laptop:
   ```bash
   ros2 topic list
   ros2 topic echo /vision/detected_objects
   rviz2
   ```

---

## 10. Troubleshooting & FAQ

### Q1: `rviz2` fails with "Could not connect to display"
**Fix**: On your host laptop terminal, run:
```bash
xhost +local:root
```
Then re-run `./docker/run.sh`.

### Q2: Permission denied when running `./docker/build.sh` or `./docker/run.sh`
**Fix**: Ensure execution permissions are set on the host:
```bash
chmod +x docker/*.sh
```

### Q3: My newly plugged-in webcam/USB device isn't showing up inside Docker
**Fix**: Docker cannot detect devices plugged in *after* the container has already booted. Simply type `exit` in the container and start it again with `./docker/run.sh`.

### Q4: How do I install a new system package inside the container permanently?
**Fix**: Add the package name to the `apt-get install` list inside [`docker/Dockerfile`](docker/Dockerfile), then run `./docker/build.sh`.

---

## 11. Command Cheat Sheet

### 🐳 ROS 2 Operations (STRICTLY INSIDE DOCKER CONTAINER)
| Action | Command | Environment & Folder |
| :--- | :--- | :--- |
| **Start / Enter container shell** | `./docker/run.sh` | Host Terminal (`~/Projects/Sem-5-mini-project`) |
| **Attach another shell to container** | `docker exec -it xarm_sorting_env bash` | Host Terminal Tab (`~/Projects/Sem-5-mini-project`) |
| **Build ROS 2 packages** | `colcon build --symlink-install` | Inside Container (`/workspace/ros2_ws`) |
| **Source workspace overlay** | `source install/setup.bash` | Inside Container (`/workspace/ros2_ws`) |
| **Run ROS 2 tests (Interactive)** | `python3 -m pytest src/sorting_arm/test/test_kinematics.py` | Inside Container (`/workspace/ros2_ws`) |
| **Run ROS 2 tests (One-Shot)** | `./docker/run.sh pytest ros2_ws/src/sorting_arm/test/test_kinematics.py` | Host Terminal (`~/Projects/Sem-5-mini-project`) |
| **Launch Digital Twin Simulation** | `ros2 launch sorting_arm simulation.launch.py gui:=true` | Inside Container (`/workspace`) |
| **Launch Full Physical Workcell** | `ros2 launch sorting_arm xarm.launch.py auto_start:=true` | Inside Container (`/workspace`) |
| **Launch xArm Hardware Driver** | `ros2 launch sorting_arm xarm_driver.launch.py` | Inside Container (`/workspace`) |
| **Run 3D Visualizer (RViz2)** | `rviz2` | Inside Container (`/workspace`) |

### 🐍 Standalone Python Operations (STRICTLY IN PYTHON VIRTUAL ENVIRONMENT ON HOST)
| Action | Command | Environment & Folder |
| :--- | :--- | :--- |
| **Setup Virtual Environment** | `./setup_venv.sh` | Host Terminal (`~/Projects/Sem-5-mini-project`) |
| **Activate Virtual Environment** | `source venv/bin/activate` | Host Terminal (`~/Projects/Sem-5-mini-project`) |
| **Run Unit Tests via pytest** | `pytest ros2_ws/src/sorting_arm/test/` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Run Offline Kinematics Tests** | `python3 ros2_ws/src/sorting_arm/test/run_tests.py` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Run Web Mission Control Dashboard** | `python3 tools/autonomous_dashboard.py --dry-run` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Run Web Hardware Config Server** | `python3 tools/web_config/server.py` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Run Terminal Autonomous Sorter** | `python3 tools/run_autonomous_system.py --dry-run --camera 0` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Calibrate Camera / ArUco Warp** | `python3 vision/test_aruco_warp.py --synthetic` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |
| **Camera Optical Leveling HUD** | `python3 vision/align_camera.py --synthetic --save-report` | Host Terminal (`venv`, `~/Projects/Sem-5-mini-project`) |

