# 🤖 Dual xArm 1s Color Sorting Robot

An autonomous robotics system that uses two xArm 1s robotic arms and an overhead camera to detect, pick, and sort colored cubes into their respective collection bins.

---

## 📌 Project Overview

This project is designed to automate color-based object sorting using computer vision and robotic manipulation.

An overhead USB camera captures the arena, detects colored cubes using OpenCV, and sends their positions to a ROS 2 scheduler. The scheduler assigns tasks to two xArm robotic arms based on their respective work zones. Each arm picks the cubes and places them into the correct compartment of the **2×2 collection box**.

---

## ✨ Features

- Dual robotic arm coordination
- OpenCV based color detection
- ArUco marker based arena calibration
- ROS 2 based architecture
- Collision-free task scheduling
- Automatic object assignment
- Real-time web dashboard
- Modular and scalable design

---

## 🛠 Hardware Used

- 2 × xArm 1s Robotic Arms
- Logitech USB Camera
- Jetson Nano / Raspberry Pi 5
- Wooden Color Cubes (4 Colors)
- **2×2 Collection Box**

---

## 💻 Software Stack

- Python
- ROS 2 Humble
- OpenCV
- xArm ROS 2 SDK
- HTML
- CSS
- JavaScript
- Git & GitHub

---

## 🎨 Supported Colors

- 🔴 Red
- 🟢 Green
- 🔵 Blue
- 🟡 Yellow

---

## 🏗 System Architecture

```
Camera
   ↓
OpenCV Vision
   ↓
Object Detection
   ↓
ROS 2 Scheduler
   ↓
Task Assignment
   ↓
xArm 1 / xArm 2
   ↓
2×2 Collection Box
```

---

## 📂 Project Structure

```
project/
│
├── vision/
├── ros2_ws/
├── dashboard/
├── config/
├── README.md
└── LICENSE
```

---

## 🚀 Installation

Clone the repository

```bash
git clone <repository-url>
```

Open the project

```bash
cd project-name
```

Install dependencies

```bash
pip install -r requirements.txt
```

Build ROS workspace

```bash
colcon build
```

Source ROS

```bash
source install/setup.bash
```

Launch the project

```bash
ros2 launch <package_name> <launch_file>
```

---

## 👨‍💻 Team Members

- Dhruv Arora
- Shubham Jain
- Karan Pal

---

## 📅 Project Status

🚧 Under Development

Current Progress

- Arena Design ✔
- System Planning ✔
- Software Architecture ✔
- Vision Pipeline 🔄
- ROS Integration ⏳
- Dual Arm Integration ⏳
- Dashboard ⏳

---

## 📄 License

This project is developed for academic purposes at GLA University, Mathura.
