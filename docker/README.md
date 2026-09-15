# 🐳 Project-Isolated ROS 2 Humble Docker Environment

This directory contains the container configuration for the **Dual xArm 1s Color-Sorting Robot**. It provides a fully isolated **ROS 2 Humble** and **OpenCV** workspace without requiring a global ROS installation on your host operating system.

---

## 🌟 Why Project-Level Docker?

1. **Host OS Independence**: Allows developing on Ubuntu 24.04, Ubuntu 26.04, or any Linux distribution without system conflicts.
2. **Multi-Project Isolation**: Run ROS 2 Humble here while other projects on your machine use ROS 2 Jazzy, Rolling, or ROS 1.
3. **Hardware Acceleration**: Automatically maps your laptop's Intel GPU (`/dev/dri`) for smooth RViz2 rendering and OpenCV GUI windows.
4. **Non-Root Ownership**: The container matches your host user ID (`UID:GID = 1000:1000`), meaning generated build files in `ros2_ws/` remain directly editable on your host without `sudo`.
5. **Cross-Platform Compatibility**: Uses multi-architecture base images (`linux/amd64` and `linux/arm64`), making it 100% portable between your x86 laptop and NVIDIA Jetson.

---

## 💻 Usage on Your Laptop

### 1. Build the Docker Image
From the project root:
```bash
./docker/build.sh
```
*This downloads ROS 2 Humble Desktop, OpenCV, Mesa OpenGL drivers, and configures the `developer` user.*

### 2. Run the Interactive Container
```bash
./docker/run.sh
```
*Your project directory is mounted inside at `/workspace`.*

### 3. Build & Test ROS 2 Inside the Container
Once inside the container shell:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Test GUI & Hardware Acceleration
Inside the container, test that RViz2 and Intel GPU rendering work:
```bash
rviz2
```
or test OpenGL rendering info:
```bash
glxinfo -B
```

---

## ⚡ Deployment to NVIDIA Jetson (ARM64)

The same Docker configuration works natively on JetPack (Jetson Orin Nano, Orin NX, AGX Orin, Xavier):

1. **Clone the repository on the Jetson**:
   ```bash
   git clone <repo-url>
   cd Sem-5-mini-project
   ```

2. **Build the container on the Jetson**:
   ```bash
   ./docker/build.sh
   ```
   *Docker on the Jetson will automatically pull the ARM64 version of ROS 2 Humble.*

3. **Run the container on the Jetson**:
   ```bash
   ./docker/run.sh
   ```
   *USB-to-UART adapters (`/dev/ttyUSB*` or `/dev/xarm_*`) and video cameras (`/dev/video*`) are automatically passed into the container.*

---

## 🌐 Distributed Operation (Laptop + Jetson)

Because both the laptop and Jetson run containers using `--net=host`, ROS 2 topics and services automatically communicate across your local Wi-Fi or Ethernet network using DDS discovery:

1. **Set matching ROS Domain IDs (optional, default is 0)**:
   ```bash
   export ROS_DOMAIN_ID=42
   ```
2. **Launch Controllers on Jetson**:
   ```bash
   # On the Jetson (inside container):
   ros2 launch sorting_scheduler system.launch.py
   ```
3. **Launch Monitoring on Laptop**:
   ```bash
   # On your laptop (inside container):
   rviz2
   # or
   ros2 topic echo /vision/detected_objects
   ```
   *The laptop immediately receives telemetry and displays the live robot state without running the physical arm drivers locally!*
